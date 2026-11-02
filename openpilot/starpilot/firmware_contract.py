"""SP-032 live updater contract; local preparation, not installation-qualified.

Separate read-only endpoints identify native-ready and pending-blocked states.
Only the native-ready endpoint proves a child passed startup acknowledgement. Kernel peer credentials, boot/
PID/start identities and loaded coordination code are checked independently.
This is generation checking for cooperative updates, not hostile-root attestation.
The installer must still hold the writer lease and apply its normal fresh guards.
"""
import dataclasses
import hashlib
import json
import os
from pathlib import Path
import re
import secrets
import stat
import socket
import struct
import sys
import time
import types

VERSION = 1
ADDRESS_PREFIX = "\0starpilot-fw-contract-v1-"
MAX_PACKET = 16384

class ContractError(RuntimeError):
  pass

def require(ok, message):
  if not ok:
    raise ContractError(message)

def file_sha256(path):
  digest = hashlib.sha256()
  with open(path, "rb") as stream:
    for block in iter(lambda: stream.read(1024 * 1024), b""):
      digest.update(block)
  return digest.hexdigest()

def boot_id():
  value = Path("/proc/sys/kernel/random/boot_id").read_text().strip()
  require(re.fullmatch("[0-9a-f]{8}(-[0-9a-f]{4}){3}-[0-9a-f]{12}", value), "Invalid boot identity")
  return value

def _stat_identity(pid):
  require(type(pid) is int and pid > 0, "Invalid PID")
  path = Path("/proc") / str(pid)
  raw = (path / "stat").read_text()
  fields = raw[raw.rfind(") ") + 2:].split()
  require(len(fields) >= 20 and fields[0] not in ("Z", "X", "x"), "Process is not live")
  return dict(pid=pid, start_ticks=int(fields[19]), ppid=int(fields[1]), uid=path.stat().st_uid)

def process_identity(pid):
  before = _stat_identity(pid)
  executable = Path("/proc") / str(pid) / "exe"
  result = dict(before, exe_sha256=file_sha256(executable))
  require(_stat_identity(pid) == before, "Process changed while observing it")
  return result

def _encoded(value, depth=0):
  require(depth < 24, "Unsupported recursive code value")
  if value is None or type(value) in (str, int, bool):
    return value
  if type(value) is float:
    return {"float": value.hex()}
  if type(value) is bytes:
    return {"bytes": value.hex()}
  if isinstance(value, types.CodeType):
    # Exclude filename and line numbers; keep behavior/exception tables and names.
    return {name: _encoded(getattr(value, name), depth + 1) for name in (
      "co_code", "co_consts", "co_names", "co_varnames", "co_freevars", "co_cellvars",
      "co_argcount", "co_posonlyargcount", "co_kwonlyargcount", "co_nlocals",
      "co_stacksize", "co_flags", "co_exceptiontable", "co_name", "co_qualname")}
  if isinstance(value, types.FunctionType):
    return {"code": _encoded(value.__code__, depth + 1),
            "defaults": _encoded(value.__defaults__, depth + 1),
            "kwdefaults": _encoded(value.__kwdefaults__, depth + 1),
            "closure": [_encoded(cell.cell_contents, depth + 1) for cell in value.__closure__ or ()]}
  if isinstance(value, (tuple, list)):
    return [_encoded(item, depth + 1) for item in value]
  if isinstance(value, (set, frozenset)):
    return {"set": sorted((_encoded(item, depth + 1) for item in value), key=lambda item: json.dumps(item, sort_keys=True))}
  if type(value) is dict:
    require(all(type(key) is str for key in value), "Unsupported code dictionary")
    return {key: _encoded(item, depth + 1) for key, item in sorted(value.items())}
  if dataclasses.is_dataclass(value) and not isinstance(value, type):
    return {"dataclass": type(value).__qualname__, "value": _encoded(dataclasses.asdict(value), depth + 1)}
  raise ContractError("Unsupported loaded code value: " + type(value).__name__)

def loaded_code_sha256(modules):
  require(set(modules) == {"updater", "barrier", "startup", "contract"}, "Missing contract module")
  updater, barrier, startup, contract = (modules[name] for name in ("updater", "barrier", "startup", "contract"))
  require(getattr(updater, "FirmwareBarrier", None) is barrier.FirmwareBarrier and getattr(updater, "NativeStartup", None) is startup.NativeStartup and
          getattr(updater, "serve_runtime", None) is contract.serve_runtime and getattr(updater, "serve_blocked_runtime", None) is contract.serve_blocked_runtime and getattr(updater, "file_sha256", None) is contract.file_sha256,
          "Updater helper binding changed")
  result = {}
  for label, module in sorted(modules.items()):
    items = {}
    for name, value in sorted(vars(module).items()):
      if isinstance(value, types.FunctionType) and value.__globals__ is vars(module):
        items[name] = _encoded(value)
      elif isinstance(value, type) and value.__module__ == module.__name__:
        methods = {}
        for key, member in sorted(vars(value).items()):
          if isinstance(member, (staticmethod, classmethod)):
            member = member.__func__
          if isinstance(member, property):
            methods[key] = [_encoded(method) if method else None for method in (member.fget, member.fset, member.fdel)]
          elif isinstance(member, types.FunctionType):
            methods[key] = _encoded(member)
        items[name] = methods
      elif name.isupper() and type(value) in (str, int, bool, float, bytes, tuple, frozenset):
        items[name] = _encoded(value)
    result[label] = items
  return hashlib.sha256(json.dumps(result, sort_keys=True, separators=(",", ":")).encode()).hexdigest()

def validate_images(images):
  require(type(images) is dict and bool(images), "Missing selected image identities")
  for serial, item in images.items():
    require(type(serial) is str and re.fullmatch("[0-9a-fA-F]{24}", serial), "Invalid selected board identity")
    require(type(item) is dict and set(item) == {"filename", "sha256"}, "Invalid selected image record")
    require(type(item["filename"]) is str and re.fullmatch("[A-Za-z0-9_]+[.]bin[.]signed", item["filename"]), "Invalid image filename")
    require(type(item["sha256"]) is str and re.fullmatch("[0-9a-f]{64}", item["sha256"]), "Invalid image hash")

def peer_identity(connection):
  return struct.unpack("3i", connection.getsockopt(socket.SOL_SOCKET, socket.SO_PEERCRED, struct.calcsize("3i")))

def runtime_snapshot(process, modules, images, stop_requested):
  require(not stop_requested() and process.poll() is None, "Native child is unavailable")
  wrapper = process_identity(os.getpid())
  child = process_identity(process.pid)
  require(child["ppid"] == wrapper["pid"] and child["uid"] == wrapper["uid"], "Native child ownership changed")
  loaded = loaded_code_sha256(modules)
  require(not stop_requested() and process.poll() is None, "Native readiness lost")
  return dict(version=VERSION, boot_id=boot_id(), wrapper=wrapper, child=child,
              loaded_sha256=loaded, python_tag=sys.implementation.cache_tag,
              images=images, observed_boottime_ns=time.clock_gettime_ns(time.CLOCK_BOOTTIME))

def serve_runtime(process, modules, images, stop_requested):
  """Caller reached this only after the real native ACK and read-lease exit."""
  validate_images(images)
  frozen_images = json.loads(json.dumps(images))
  with socket.socket(socket.AF_UNIX, socket.SOCK_SEQPACKET) as listener:
    listener.bind(ADDRESS_PREFIX + str(os.getpid()))
    listener.listen(4)
    listener.settimeout(.1)
    while not stop_requested() and process.poll() is None:
      try:
        connection, _ = listener.accept()
      except socket.timeout:
        continue
      with connection:
        connection.settimeout(.2)
        try:
          _, uid, _ = peer_identity(connection)
          require(uid in (0, os.geteuid()), "Untrusted contract client")
          raw, _, flags, _ = connection.recvmsg(MAX_PACKET)
          require(not flags & socket.MSG_TRUNC, "Truncated contract request")
          request = json.loads(raw)
          require(type(request) is dict and set(request) == {"version", "nonce"} and
                  type(request["version"]) is int and request["version"] == VERSION and
                  type(request["nonce"]) is str and re.fullmatch("[0-9a-f]{32}", request["nonce"]),
                  "Invalid contract request")
          reply = runtime_snapshot(process, modules, frozen_images, stop_requested)
          reply["nonce"] = request["nonce"]
          packet = json.dumps(reply, sort_keys=True, separators=(",", ":")).encode()
          require(len(packet) <= MAX_PACKET, "Oversized contract response")
          require(connection.send(packet) == len(packet), "Incomplete contract response")
        except (OSError, ValueError, ContractError):
          # A failed observer never authorizes writes or terminates a healthy child.
          continue

def scan_runtime_processes(root):
  """Find known updater/native entry points across checkouts; fail on unreadable proc."""
  root = Path(root)
  require(root.is_absolute() and root.resolve() == root, "Noncanonical source root")
  wrapper_path = str(root / "openpilot/selfdrive/pandad/pandad.py")
  result = set()
  for path in Path("/proc").iterdir():
    if not path.name.isdecimal():
      continue
    try:
      argv = (path / "cmdline").read_bytes().split(b"\0")
      args = [part.decode("utf-8", "strict") for part in argv if part]
      exe = os.readlink(path / "exe").removesuffix(" (deleted)")
      role = None
      if any(arg == "openpilot.selfdrive.pandad.pandad" or arg == wrapper_path or arg.endswith("/selfdrive/pandad/pandad.py") for arg in args):
        role = "wrapper"
      if Path(exe).name == "pandad":
        require(role is None, "Ambiguous updater role")
        role = "native"
      if role:
        _stat_identity(int(path.name))
        result.add((int(path.name), role))
    except FileNotFoundError:
      continue
  return result

def query_runtime(wrapper_pid, expected, source_root, *, timeout=2.0):
  """Use inside exclusive writer admission, plus fresh device/manager guards.

  expected is bound to the prepared package and fresh manager observation.
  This function neither obtains that lease nor supplies those device guards.
  """
  require(set(expected) == {"boot_id", "wrapper", "native_sha256", "loaded_sha256", "python_tag", "images"}, "Incomplete contract expectation")
  require(set(expected["wrapper"]) == {"pid", "ppid", "start_ticks", "uid"} and
          expected["wrapper"]["pid"] == wrapper_pid, "Invalid wrapper expectation")
  validate_images(expected["images"])
  require(0 < timeout <= 5, "Invalid contract timeout")
  nonce = secrets.token_hex(16)
  begin = time.clock_gettime_ns(time.CLOCK_BOOTTIME)
  try:
    with socket.socket(socket.AF_UNIX, socket.SOCK_SEQPACKET) as connection:
      connection.settimeout(timeout)
      connection.connect(ADDRESS_PREFIX + str(wrapper_pid))
      pid, uid, _ = peer_identity(connection)
      require(pid == wrapper_pid and uid == expected["wrapper"]["uid"], "Contract endpoint has wrong owner")
      connection.sendall(json.dumps(dict(version=VERSION, nonce=nonce)).encode())
      raw, _, flags, _ = connection.recvmsg(MAX_PACKET)
      require(not flags & socket.MSG_TRUNC, "Truncated contract response")
      reply = json.loads(raw)
    end = time.clock_gettime_ns(time.CLOCK_BOOTTIME)
    require(type(reply) is dict and set(reply) == {"version", "boot_id", "wrapper", "child", "loaded_sha256",
            "python_tag", "images", "observed_boottime_ns", "nonce"}, "Malformed contract response")
    require(reply["version"] == VERSION and reply["nonce"] == nonce and
            type(reply["observed_boottime_ns"]) is int and begin <= reply["observed_boottime_ns"] <= end,
            "Stale or unbound contract response")
    require(reply["boot_id"] == expected["boot_id"] == boot_id(), "Boot changed")
    for key in ("loaded_sha256", "python_tag", "images"):
      require(reply[key] == expected[key], "Running contract differs: " + key)
    require({key: reply["wrapper"].get(key) for key in expected["wrapper"]} == expected["wrapper"], "Wrapper generation changed")
    require(reply["wrapper"] == process_identity(wrapper_pid), "Wrapper observation changed")
    require(reply["child"] == process_identity(reply["child"]["pid"]), "Native observation changed")
    require(reply["child"]["ppid"] == wrapper_pid and reply["child"]["uid"] == uid and
            reply["child"]["exe_sha256"] == expected["native_sha256"], "Wrong native generation")
    wanted = {(wrapper_pid, "wrapper"), (reply["child"]["pid"], "native")}
    require(scan_runtime_processes(source_root) == wanted, "Old, duplicate or missing updater/native processes")
    require(process_identity(wrapper_pid) == reply["wrapper"] and
            process_identity(reply["child"]["pid"]) == reply["child"], "Runtime changed during admission")
    return reply
  except (OSError, ValueError, KeyError, TypeError) as error:
    raise ContractError("Runtime contract unavailable or malformed") from error

# Separate endpoint: blocked startup is never native readiness or recovery permission.
BLOCKED_ADDRESS_PREFIX = "\0starpilot-fw-blocked-v1-"
BLOCKED_KIND = 'blocked_on_pending'

def blocked_pending_identity(barrier):
  """Observe canonical pending intent without acquiring a competing lock.

  The recovery client holds EX while querying. Every path is checked against its
  opened identity; no directory, mutex or intent is created or changed here.
  """
  directory=barrier._directory();mutex=pending=None
  try:
    mutex=os.open('mutex',os.O_RDONLY|os.O_NOFOLLOW|os.O_CLOEXEC,dir_fd=directory)
    lock_info=os.fstat(mutex)
    require(stat.S_ISREG(lock_info.st_mode) and lock_info.st_nlink==1 and lock_info.st_uid==barrier.owner_uid and stat.S_IMODE(lock_info.st_mode)==0o600,'Untrusted blocked mutex')
    pending=os.open('pending.json',os.O_RDONLY|os.O_NOFOLLOW|os.O_CLOEXEC,dir_fd=directory)
    info=os.fstat(pending)
    require(stat.S_ISREG(info.st_mode) and info.st_nlink==1 and info.st_uid in (0,barrier.owner_uid) and stat.S_IMODE(info.st_mode)==0o600,'Untrusted blocked intent')
    raw=os.read(pending,4097);value=json.loads(raw)
    require(type(value) is dict and set(value)=={'version','token','manifest'} and type(value['version']) is int and value['version']==1,'Malformed blocked intent')
    require(type(value['token']) is str and re.fullmatch('[0-9a-f]{32}',value['token']) and type(value['manifest']) is str and re.fullmatch('[0-9a-f]{64}',value['manifest']),'Invalid blocked transaction identity')
    canonical=(json.dumps(value,sort_keys=True,separators=(',',':'))+'\n').encode()
    require(raw==canonical,'Noncanonical blocked intent')
    def identity(item):return item.st_dev,item.st_ino,item.st_uid,item.st_mode,item.st_nlink
    require(identity(os.stat(barrier.directory,follow_symlinks=False))==identity(os.fstat(directory)),'Blocked directory changed')
    require(identity(os.stat('mutex',dir_fd=directory,follow_symlinks=False))==identity(lock_info),'Blocked mutex changed')
    require(identity(os.stat('pending.json',dir_fd=directory,follow_symlinks=False))==identity(info) and os.fstat(pending).st_size==len(raw),'Blocked intent changed')
    return dict(token=value['token'],manifest=value['manifest'],sha256=hashlib.sha256(raw).hexdigest())
  finally:
    if pending is not None:os.close(pending)
    if mutex is not None:os.close(mutex)
    os.close(directory)

def blocked_runtime_snapshot(barrier,modules,stop_requested):
  require(not stop_requested(),'Blocked updater is stopping')
  with barrier.reader() as lease:
    require(not lease,'Updater is no longer blocked')
  pending=blocked_pending_identity(barrier)
  wrapper=process_identity(os.getpid())
  loaded=loaded_code_sha256(modules)
  require(not stop_requested() and pending==blocked_pending_identity(barrier),'Blocked intent changed during observation')
  return dict(version=VERSION,kind=BLOCKED_KIND,boot_id=boot_id(),wrapper=wrapper,
              loaded_sha256=loaded,python_tag=sys.implementation.cache_tag,
              pending=pending,barrier_directory=str(barrier.directory),
              observed_boottime_ns=time.clock_gettime_ns(time.CLOCK_BOOTTIME),
              native_ready=False,recovery_authorized=False)

def serve_blocked_runtime(barrier,modules,stop_requested):
  """Called outside reader() with no native child; never touches Panda/intent."""
  with socket.socket(socket.AF_UNIX,socket.SOCK_SEQPACKET) as listener:
    listener.bind(BLOCKED_ADDRESS_PREFIX+str(os.getpid()));listener.listen(4);listener.settimeout(.1)
    while not stop_requested():
      # Release the shared lease before listening. A recovery writer must be able
      # to acquire EX; after it clears verified intent, main reacquires a reader.
      # An existing intent already inhibits startup. Do not repeatedly take SH
      # while a recovery client needs EX. Absence alone never permits startup:
      # reader() rechecks both the mutex and pending state before returning.
      directory=barrier._directory()
      try:pending=barrier._pending(directory)
      finally:os.close(directory)
      if not pending:
        with barrier.reader() as lease:
          if lease:return
      try:connection,_=listener.accept()
      except socket.timeout:continue
      with connection:
        connection.settimeout(.2)
        try:
          _,uid,_=peer_identity(connection);require(uid in (0,os.geteuid()),'Untrusted blocked client')
          raw,_,flags,_=connection.recvmsg(MAX_PACKET)
          require(not flags&socket.MSG_TRUNC,'Truncated blocked request')
          request=json.loads(raw)
          require(type(request) is dict and set(request)=={'version','kind','nonce'} and type(request['version']) is int and request['version']==VERSION and request['kind']==BLOCKED_KIND and type(request['nonce']) is str and re.fullmatch('[0-9a-f]{32}',request['nonce']),'Invalid blocked request')
          reply=blocked_runtime_snapshot(barrier,modules,stop_requested);reply['nonce']=request['nonce']
          packet=json.dumps(reply,sort_keys=True,separators=(',',':')).encode()
          require(len(packet)<=MAX_PACKET and connection.send(packet)==len(packet),'Incomplete blocked response')
        except (OSError,ValueError,ContractError):continue

def query_blocked_runtime(wrapper_pid,expected,source_root,*,timeout=2.0):
  """Read-only proof for a recovery admission callback already holding EX.

  This proves cooperative blocked code and absence of known native/updater
  duplicates only. It does not supply parked health, ownership rebind or permission
  to restore files, clear intent, restart, flash or recover the vehicle.
  """
  require(type(expected) is dict and set(expected)=={'boot_id','wrapper','loaded_sha256','python_tag','pending','barrier_directory'},'Incomplete blocked expectation')
  require(type(expected['wrapper']) is dict and set(expected['wrapper'])=={'pid','ppid','start_ticks','uid'} and expected['wrapper']['pid']==wrapper_pid,'Invalid blocked wrapper expectation')
  require(type(expected['pending']) is dict and set(expected['pending'])=={'token','manifest','sha256'},'Invalid blocked intent expectation')
  require(type(expected['barrier_directory']) is str and Path(expected['barrier_directory']).is_absolute(),'Invalid blocked barrier expectation')
  require(0<timeout<=5,'Invalid blocked timeout')
  nonce=secrets.token_hex(16);begin=time.clock_gettime_ns(time.CLOCK_BOOTTIME)
  try:
    with socket.socket(socket.AF_UNIX,socket.SOCK_SEQPACKET) as connection:
      connection.settimeout(timeout);connection.connect(BLOCKED_ADDRESS_PREFIX+str(wrapper_pid))
      pid,uid,_=peer_identity(connection)
      require(pid==wrapper_pid and uid==expected['wrapper']['uid'],'Blocked endpoint has wrong owner')
      connection.sendall(json.dumps(dict(version=VERSION,kind=BLOCKED_KIND,nonce=nonce)).encode())
      raw,_,flags,_=connection.recvmsg(MAX_PACKET);require(not flags&socket.MSG_TRUNC,'Truncated blocked response');reply=json.loads(raw)
    end=time.clock_gettime_ns(time.CLOCK_BOOTTIME)
    require(type(reply) is dict and set(reply)=={'version','kind','boot_id','wrapper','loaded_sha256','python_tag','pending','barrier_directory','observed_boottime_ns','native_ready','recovery_authorized','nonce'},'Malformed blocked response')
    require(type(reply['version']) is int and reply['version']==VERSION and reply['kind']==BLOCKED_KIND and reply['native_ready'] is False and reply['recovery_authorized'] is False,'Blocked proof is not permission or native readiness')
    require(reply['nonce']==nonce and type(reply['observed_boottime_ns']) is int and begin<=reply['observed_boottime_ns']<=end,'Stale blocked response')
    require(reply['boot_id']==expected['boot_id']==boot_id(),'Blocked boot changed')
    for key in ('loaded_sha256','python_tag','pending','barrier_directory'):
      require(reply[key]==expected[key],'Blocked runtime differs: '+key)
    require(type(reply['wrapper']) is dict and {key:reply['wrapper'].get(key) for key in expected['wrapper']}==expected['wrapper'],'Blocked wrapper generation changed')
    require(reply['wrapper']==process_identity(wrapper_pid),'Blocked wrapper observation changed')
    require(scan_runtime_processes(source_root)=={(wrapper_pid,'wrapper')},'Native, old, duplicate or missing process during blocked recovery')
    require(process_identity(wrapper_pid)==reply['wrapper'],'Blocked wrapper changed during admission')
    return reply
  except (OSError,ValueError,KeyError,TypeError) as error:
    raise ContractError('Blocked runtime unavailable or malformed') from error