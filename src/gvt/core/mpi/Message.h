/* =======================================================================================
   This file is released as part of GraviT - scalable, platform independent ray
   tracing
   tacc.github.io/GraviT

   Copyright 2013-2015 Texas Advanced Computing Center, The University of Texas
   at Austin
   All rights reserved.

   Licensed under the BSD 3-Clause License, (the "License"); you may not use
   this file
   except in compliance with the License.
   A copy of the License is included with this software in the file LICENSE.
   If your copy does not contain the License, you may obtain a copy of the
   License at:

       http://opensource.org/licenses/BSD-3-Clause

   Unless required by applicable law or agreed to in writing, software
   distributed under
   the License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY
   KIND, either express or implied.
   See the License for the specific language governing permissions and
   limitations under
   limitations under the License.

   GraviT is funded in part by the US National Science Foundation under awards
   ACI-1339863,
   ACI-1339881 and ACI-1339840
   =======================================================================================
   */

//
// Message.h
//

#ifndef GVT_CORE_MPI_MESSAGE_H
#define GVT_CORE_MPI_MESSAGE_H

#include <mpi.h>
#include <stdlib.h>
#include "gvt/core/mpi/Work.h"

namespace gvt {
namespace core {
namespace mpi {

class MessageManager {
public:
  MessageManager();
  ~MessageManager();

  void Wait();

  void Initialize();
  void Start();

  int GetSize() { return mpi_size; }
  int GetRank() { return mpi_rank; }

private:
  static void *messageThread(void *);

  pthread_mutex_t lock;
  pthread_cond_t cond;
  pthread_t tid;

  int wait;
  int mpi_rank;
  int mpi_size;
};

class Message {
  friend class MessageManager;

public:
  static const int HEADER_TAG = 1;
  static const int BODY_TAG = 2;

  // Point to point, fire-and-forget
  Message(Work *w, int destination);

  // One-to-all, may be collective (that is, may be run in MPI thread), may be blocking
  Message(Work *w, bool collective = false, bool blocking = false);

  Message();
  ~Message();

  bool IsReady();

  void Enqueue();

  virtual void Send();
  virtual void Receive();

  int GetType() { return header.type; }
  size_t GetSize() { return header.size; }
  unsigned char *GetBytes() { return serialized; }

  // Wait for message to be handled by MPI thread
  // If the message implies a collective, it'll therefore
  // be waiting until the collective completes.
  // ONLY VALID IF BLOCKING

  void Wait();

protected:
  struct {
    int broadcast_root;
    int type;
    int destination;
    int source;
    size_t size;
    bool collective;
  } header;

  int id;
  int pending;

  bool blocking;
  pthread_mutex_t lock;
  pthread_cond_t cond;

  unsigned char *serialized;
  MPI_Status status;
  MPI_Request request;
};

} // ns mpi
} // ns core
} // ns gvt

#endif /* GVT_CORE_MPI_MESSAGE_H */