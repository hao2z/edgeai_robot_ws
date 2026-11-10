#pragma once
#include <array>
#include <chrono>
#include <cmath>
#include <csignal>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <spawn.h>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>
#include <fcntl.h>
#include <sys/wait.h>
#include <unistd.h>
#include <nlohmann/json.hpp>
extern char ** environ;
namespace fs = std::filesystem;
using Json = nlohmann::json;
using Clock = std::chrono::steady_clock;
using Pose = std::array<double, 3>;
static volatile std::sig_atomic_t interrupted = 0;

Json readJson(const fs::path & path)
{std::ifstream f(path); if (!f) {throw std::runtime_error("Cannot read " + path.string());} Json j; f >> j; return j;}
void writeJson(const fs::path & path, const Json & data)
{
  auto tmp = path; tmp += ".tmp";
  {std::ofstream f(tmp); f << data.dump(2); if (!f) {throw std::runtime_error("Write failed: " + tmp.string());}}
  fs::rename(tmp, path);
}
Pose compose(const Pose & a, const Pose & b)
{
  return {a[0]+std::cos(a[2])*b[0]-std::sin(a[2])*b[1],
    a[1]+std::sin(a[2])*b[0]+std::cos(a[2])*b[1],
    std::atan2(std::sin(a[2]+b[2]), std::cos(a[2]+b[2]))};
}
Pose inverse(const Pose & p)
{return {-std::cos(p[2])*p[0]-std::sin(p[2])*p[1], std::sin(p[2])*p[0]-std::cos(p[2])*p[1], -p[2]};}

std::string processStart(pid_t pid)
{
  std::ifstream file(fs::path("/proc")/std::to_string(pid)/"stat");std::string line,value;
  if(!std::getline(file,line)) {throw std::runtime_error("Cannot identify child process");}
  std::istringstream fields(line.substr(line.rfind(')')+2));
  for(int i=3;i<=22;++i) {fields>>value;}
  return value;
}

// No shell parsing, fork, or blocking wait in the normal tree tick path.
class Child
{
public:
  pid_t pid = -1;
  int status = 0;
  bool sent_stop = false;
  Clock::time_point stop_at;
  ~Child() {stop();}
  void launch(std::vector<std::string> argv, const fs::path & log)
  {
    if (pid > 0) {throw std::runtime_error("Child already active");}
    posix_spawn_file_actions_t actions;
    posix_spawnattr_t attr;
    posix_spawn_file_actions_init(&actions); posix_spawnattr_init(&attr);
    posix_spawn_file_actions_addopen(&actions, STDIN_FILENO, "/dev/null", O_RDONLY, 0);
    posix_spawn_file_actions_addopen(&actions, STDOUT_FILENO, log.c_str(), O_WRONLY|O_CREAT|O_APPEND, 0644);
    posix_spawn_file_actions_adddup2(&actions, STDOUT_FILENO, STDERR_FILENO);
    posix_spawnattr_setflags(&attr, POSIX_SPAWN_SETPGROUP);
    posix_spawnattr_setpgroup(&attr, 0);
    std::vector<char *> args;
    for (auto & s : argv) {args.push_back(s.data());} args.push_back(nullptr);
    int error = posix_spawnp(&pid, args[0], &actions, &attr, args.data(), environ);
    posix_spawn_file_actions_destroy(&actions); posix_spawnattr_destroy(&attr);
    if (error) {pid = -1; throw std::runtime_error("Stage spawn failed: " + std::to_string(error));}
    sent_stop = false;
  }
  bool done()
  {
    if (pid <= 0) {return true;}
    auto r = waitpid(pid, &status, WNOHANG);
    if (r == pid) {pid = -1; return true;}
    if (r < 0) {throw std::runtime_error("Lost child process status");}
    return false;
  }
  bool success() const {return WIFEXITED(status) && WEXITSTATUS(status) == 0;}
  void requestStop()
  {
    if (pid > 0 && !sent_stop) {kill(-pid, SIGTERM); sent_stop = true; stop_at = Clock::now();}
  }
  bool stopped()
  {
    if (done()) {return true;}
    if (sent_stop && Clock::now()-stop_at > std::chrono::seconds(15)) {
      kill(-pid, SIGKILL);
    }
    return false;
  }
  void stop()
  {
    requestStop();
    // Only cancellation/shutdown uses this bounded wait, never a running tick.
    while (!stopped()) {std::this_thread::sleep_for(std::chrono::milliseconds(20));}
  }
};
