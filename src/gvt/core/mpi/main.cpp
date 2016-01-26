/*THIS FILE SHOULD BE REMOVED */

#include <stdlib.h>
#include <mpi.h>
#include <unistd.h>
#include <pthread.h>
#include <iostream>
#include <sstream>

#include "Application.h"
#include "main.h"

using namespace std;

// cute little debugger setup that starts lldb in an xterm for a selected set of processes
void setup_debugger(int, char **);

WORK_CLASS(Ping) // Create a work-unit class called "Ping"

bool Ping::Action() {
  // if I am the last process in the loop, print the result

  int pingrank;
  int pingsize;
  pingrank = Application::GetApplication()->GetRank();
  pingsize = Application::GetApplication()->GetSize();
  if (Application::GetApplication()->GetRank() == (Application::GetApplication()->GetSize() - 1)) {
    std::cerr << (this->GetValue() + 1) << " workers " << std::endl;
  } else {
    // bump the count and pass on the ping

    Ping ping;
    ping.SetValue(this->GetValue() + 1);
    ping.Send(Application::GetApplication()->GetRank() + 1);
  }

  // Do not quit the message loop

  return false;
}

WORK_CLASS(AllG) // Create a work-unit class called 'AllG'

bool AllG::Action() {
  // Do an Allgather that fills a buffer with the ranks of the processes

  int me = Application::GetApplication()->GetRank();
  int buf[Application::GetApplication()->GetSize()];

  MPI_Allgather((const void *)&me, 1, MPI_INT, (void *)buf, 1, MPI_INT, MPI_COMM_WORLD);

  // If the root process, print the buffer

  if (Application::GetApplication()->GetRank() == 0) {
    for (int i = 0; i < Application::GetApplication()->GetSize(); i++) std::cerr << buf[i] << " ";
    std::cerr << "\n";
  }

  // Do not quit the message loop

  return false;
}

int main(int argc, char *argv[]) {

  Application theApplication(&argc, &argv);
  //  setup_debugger(argc, argv);

  Ping::Register(); // Register Ping work-unit class
  AllG::Register(); // Register AllG work-unit class

  theApplication.Start(); // Start the various component threads

  // Here the rank=0 node kicks off a ping and a collective Allgather

  if (theApplication.GetRank() == 0) {
    Ping ping;
    ping.SetValue(1);
    ping.Send(1); // Ping is sent to node 1

    AllG allg;
    allg.Broadcast(true, true); // Send to everyone - its a collective, and wait for the
                                // collective to be executed in the communications thread

    // give it a second for the ping to complete
    sleep(1);

    // Tell everyone to quit

    theApplication.QuitApplication();
  }
}

void setup_debugger(int argc, char **argv) {
  if (argc > 1 && ((atoi(argv[1]) == Application::GetApplication()->GetRank()) || (atoi(argv[1]) == -1))) {
    if (argc == 2) {
      std::cerr << getpid() << "\n";
    } else {
      std::stringstream cmd;
      cmd << "xterm -e lldb -p " << getpid();
      for (int i = 2; i < argc; i++) cmd << " " << argv[i];
      cmd << " " << argv[0] << " &";
      std::cerr << "running command: " << cmd.str() << "\n";
      system(cmd.str().c_str());
    }

    static int dbg = 1;
    while (dbg) sleep(1);
  }
}
