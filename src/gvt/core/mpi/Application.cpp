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
// Applicatoin.cpp
//

#ifndef GVT_CORE_MPI_APPLICATION_CPP
#define GVT_CORE_MPI_APPLICATION_CPP

#include "mpi.h"
#include "unistd.h"
#include "iostream"
#include <gvt/core/mpi/Application.h>

using namespace gvt::core::mpi;

WORK_CLASS(Quit)

Application *theApplication = NULL;

Application *
Application::GetApplication() { return theApplication; }

Application::Application(int *a, char ***b)
{
	theApplication = this;
	application_done = false;

	argcp = a;
	argvp = b;

	theMessageManager.Initialize();

	pthread_mutex_init(&lock, NULL);
	pthread_cond_init(&cond, NULL);

	pthread_mutex_lock(&lock);

	theIncomingQueue = new MessageQ("incoming");
  theOutgoingQueue = new MessageQ("outgoing");

	Quit::Register();
	pthread_mutex_unlock(&lock);
	//theApplication = this;
}

Application::~Application()
{
	Wait();
	delete theIncomingQueue;
	delete theOutgoingQueue;
}

void
Application::QuitApplication()
{
	Quit quit;
	quit.Broadcast(true, true);
}

void
Application::Start()
{
	GetMessageManager()->Start();

	if (pthread_create(&work_thread_tid, NULL, workThread, this))
	{
		std::cerr << "Failed to spawn work thread\n";
		exit(1);
	}

	// Wait for ping from worker thread
	pthread_cond_wait(&cond, &lock);
	//pthread_mutex_unlock(&lock);
}

void
Application::Kill()
{
	pthread_mutex_lock(&lock);
	application_done = true;

	theIncomingQueue->Kill();
	theOutgoingQueue->Kill();

	pthread_cond_broadcast(&cond);
	pthread_mutex_unlock(&lock);

	pthread_join(work_thread_tid, NULL);
}

void
Application::Wait()
{
	while (! application_done)
		pthread_cond_wait(&cond, &lock);
	pthread_mutex_unlock(&lock);
	GetMessageManager()->Wait();
}

void *
Application::workThread(void *p)
{
	Application *theApplication = (Application *)p;

	pthread_mutex_lock(&theApplication->lock);
	pthread_cond_signal(&theApplication->cond);
	pthread_mutex_unlock(&theApplication->lock);

	Message *m;
  while (theApplication->Running() && (m = theApplication->GetIncomingMessageQueue()->Dequeue()) != NULL)
  {
    Work *w = theApplication->Deserialize(m);
    delete m;

		if (w->Action())
		{
			theApplication->Kill();
		}

		delete w;
  }

	pthread_exit(NULL);
}
#endif /* GVT_CORE_MPI_APPLICATION_H */
