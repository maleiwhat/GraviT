#pragma once

#include <mpi.h>
#include <stdlib.h>
#include <iostream>
#include <memory>
#include "classes.h"
#include "Work.h"

namespace gvt {
namespace core {
namespace mpi {

class MessageManager;

class Message {
  friend class MessageManager;

public:
  // Point to point, fire-and-forget
  Message(Work *w, int i);

  // One-to-all, may be a communicating action (that is, may be run in MPI thread), may be blocking
  Message(Work *w, bool communicates, bool blocking = false);

  // Message to be read off MPI
  Message(MPI_Status &);

  Message();

  ~Message();

public:
  void Send(int i);
  void Broadcast();

  int GetType() { return header.type; }
  void SetType(int t) { header.type = t; }

  unsigned char *GetContent() { return content->get(); }
  size_t GetSize() { return content->get_size(); }

  int GetRoot() { return header.broadcast_root; }
  void SetRoot(int r) { header.broadcast_root = r; }
  void SetP2P(int r) { header.broadcast_root = false; }
  bool IsBroadcast() { return header.broadcast_root != -1; }

  bool Communicates();
  void SetCommunicates(bool b = true) { header.communicates = b; }

  bool HasContent() { return header.HasContent(); }
  void SetHasContent(bool b) { header.SetHasContent(b); }

  void SetDestination(int i) { destination = i; }
  int GetDestination() { return destination; }

  // Wait for a blocking message to be handled.  This will return
  // when the message has been sent, and if the message is a communcating
  // message (that is, runs in the communications thread), for the message's
  // action to happen locally.  ONLY VALID IF BLOCKING
  void Wait();

  SharedP ShareContent() { return content; }

protected:
  struct MessageHeader {

    int broadcast_root;
    int type;
    bool hasContent;
    bool communicates;

    bool HasContent() { return hasContent; }
    void SetHasContent(bool b) { hasContent = b; }

  } header;

  unsigned char *GetHeader() { return (unsigned char *)&header; }
  int GetHeaderSize() { return sizeof(header); }

  int id;

  bool blocking;
  pthread_mutex_t lock;
  pthread_cond_t cond;

  SharedP content;
  int destination;

  MPI_Status status;
  MPI_Request request;
};
}
}
}
