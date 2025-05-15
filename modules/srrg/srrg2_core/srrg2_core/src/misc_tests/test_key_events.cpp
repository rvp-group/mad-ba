#include <iostream>
#include <stdio.h>
#include <unistd.h>
#include <sys/select.h>
#include <map>
#include <unordered_map>
#include <string>
#include <atomic>
#include <thread>

#include "srrg_system_utils/parse_command_line.h"
#include "srrg_system_utils/shell_colors.h"

const char* banner[]={
  "test for a keyboard event handler.",
  " Useful to pause file sources",
  " Press SPACE to pause and RESUME playback",
  " The drawback is that you always have to press enter after",
  0
};


class KeyboardEventHandler{
public:
  typedef void (*CallbackFunction)(void); // function pointer type
  using KeyCallbackMap = std::unordered_map<uint8_t, CallbackFunction>;

  static constexpr int SRRG2_KEY_A        = 0xA61;
  static constexpr int SRRG2_KEY_SPACEBAR = 0xA20;

  KeyboardEventHandler() {
    /* Wait up to five seconds. */
    _tv.tv_sec = 1;
    _tv.tv_usec = 0;
    _run = true;
    _key_event_processed = true;
    _key = 0x0;
  }

  ~KeyboardEventHandler() {}

  bool installCallback(const int& key_, CallbackFunction f) {
    _callback_map.insert(std::make_pair(key_, f));
    return true;
  }

  std::shared_ptr<std::thread> start() {
    return std::make_shared<std::thread>([=] {_start();});
  }

  static void stop() {_run = false;}

protected:
  void _start() {
    while(_run) {
      _reset();
      if (FD_ISSET(fileno(stdin), &_read_fds)) {
        int size = read(fileno(stdin), &_key, sizeof(int));
        if (size<1) throw std::runtime_error("KeyboardEventHandler|invalid read");

//        printf("key %.4x\n", _key);

        auto it = _callback_map.find(_key);
        if (it != _callback_map.end()) {
          (*it->second)();
        }
      }
    }
  }

  struct timeval _tv;


  bool _key_event_processed;
  fd_set _read_fds;
  fd_set _write_fds;
  fd_set _external_fds;
  int _key;
  static std::atomic<bool> _run;

  KeyCallbackMap _callback_map;

  void _reset() {
    _key = 0;
    FD_ZERO(&_read_fds);
    FD_ZERO(&_write_fds);
    FD_ZERO(&_external_fds);
    FD_SET(fileno(stdin), &_read_fds);
    select(fileno(stdin)+1, &_read_fds, &_write_fds, &_external_fds, &_tv);
  }
};

std::atomic<bool> KeyboardEventHandler::_run(true);

//ia this will be inside the base source and there will be
//ia static methods to get and set this shit
std::atomic<bool> pause_playback;

void keyA_handler() {
  std::cerr << "you said A\n";
}

void keySPACE_handler() {
  std::cerr << "you said SPACE\n";
  if (pause_playback) {
    std::cerr << FG_YELLOW("resuming playback") << std::endl;
    pause_playback = false;
  } else {
    std::cerr << FG_YELLOW("pausing playback") << std::endl;
    pause_playback = true;
  }
}

int main(int argc, char **argv) {
  srrg2_core::ParseCommandLine cmd_line(argv,banner);
  cmd_line.parse();
  pause_playback = false;

  //ia inside your source declare one of this and register all the callbacks.
  KeyboardEventHandler eh;
  eh.installCallback(KeyboardEventHandler::SRRG2_KEY_A, keyA_handler);
  eh.installCallback(KeyboardEventHandler::SRRG2_KEY_SPACEBAR, keySPACE_handler);

  //ia start the event thread that uses the select to check if a key has been pressed.
  std::shared_ptr<std::thread> event_t = eh.start();

  //ia source go if no-one is pausing from keyboard
  size_t i = 0;
  while (i < 100) {
    std::cerr << i << "\r";
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    ++i;

    if (pause_playback) {
      std::cerr << "\nwaiting";
      while(pause_playback) {
        std::cerr << ".";
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
      }
      std::cerr << "\n";
    }
  }

  //ia once the bag is ended, stop the thread to let him join
  KeyboardEventHandler::stop();
  event_t->join();

  //ia stop
  std::cerr << "ending\n";

  return 0;
}


