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
// MessageQ.h
//

#ifndef GVT_CORE_MPI_MESSAGEQ_H
#define GVT_CORE_MPI_MESSAGEQ_H

#include <iostream>
#include <queue>
#include <pthread.h>
#include <gvt/core/mpi/Message.h>

namespace gvt {
namespace core {
namespace mpi {

class MessageQ
{
public:
	MessageQ(const char *n) : name(n)
	{
		pthread_mutex_init(&lock, NULL);
		pthread_cond_init(&signal, NULL);
		running = true;
	}
	~MessageQ(){}

	void Kill();

	void Enqueue(Message *w);
	Message *Dequeue();
	int IsReady();

private:
	const char *name;

	pthread_mutex_t lock;
	pthread_cond_t  signal;
	bool running;

	queue<Message*> workq;
};

MessageQ *GetIncomingMessageQueue();
MessageQ *GetOutgoingMessageQueue();

} //ns mpi
} //ns core
} //ns gvt

#endif /* GVT_CORE_MPI_MESSAGEQ_H */
