#pragma once
// SP-032: called only after ALL native Panda startup checks, before worker threads.
// The inherited lease protects image reads even if the Python parent dies.
// Legacy launchers without this contract remain compatible during bootstrap.
#include <cerrno>
#include <charconv>
#include <cstdlib>
#include <cstring>
#include <string>
#include <sys/socket.h>
#include <sys/stat.h>
#include <unistd.h>

namespace firmware_startup {
inline bool descriptor(const char *text, int &value) {
  if (!text || !*text) return false;
  const char *end = text + std::strlen(text);
  const auto parsed = std::from_chars(text, end, value);
  return parsed.ec == std::errc{} && parsed.ptr == end && value >= 3;
}

inline bool acknowledge() {
  const char *socket_text = std::getenv("STARPILOT_PANDAD_READY_FD");
  const char *lease_text = std::getenv("STARPILOT_PANDAD_READY_LEASE_FD");
  const char *nonce = std::getenv("STARPILOT_PANDAD_READY_NONCE");
  if (!socket_text && !lease_text && !nonce) return true;
  int socket_fd = -1, lease_fd = -1;
  if (!descriptor(socket_text, socket_fd) || !descriptor(lease_text, lease_fd) ||
      socket_fd == lease_fd || !nonce || std::strlen(nonce) != 32 ||
      std::strspn(nonce, "0123456789abcdef") != 32) return false;
  const char *managed = std::getenv("STARPILOT_PANDA_FIRMWARE_MAP");
  if (!managed || std::string(managed) != "1") return false;
  struct stat lease{};
  if (fstat(lease_fd, &lease) != 0 || !S_ISREG(lease.st_mode) || lease.st_nlink != 1 ||
      (lease.st_mode & 0777) != 0600 || lease.st_uid != geteuid()) return false;
  int type = 0;
  socklen_t size = sizeof(type);
  if (getsockopt(socket_fd, SOL_SOCKET, SO_TYPE, &type, &size) != 0 || type != SOCK_SEQPACKET) return false;
  const std::string message = "SP032_FW_READY_V1 " + std::string(nonce) + " " + std::to_string(getpid()) + "\n";
  ssize_t sent;
  do {
    sent = send(socket_fd, message.data(), message.size(), MSG_NOSIGNAL | MSG_DONTWAIT);
  } while (sent < 0 && errno == EINTR);
  // On failure retain the lease until process exit: the parent must stop us
  // before admitting another writer. Never pretend a timeout was an ACK.
  if (sent != static_cast<ssize_t>(message.size())) return false;
  close(socket_fd);
  close(lease_fd);
  unsetenv("STARPILOT_PANDAD_READY_FD");
  unsetenv("STARPILOT_PANDAD_READY_LEASE_FD");
  unsetenv("STARPILOT_PANDAD_READY_NONCE");
  return true;
}
}  // namespace firmware_startup
