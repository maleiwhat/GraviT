#ifndef GVT_CORE_MPI_APPLICATION_CPP
#define GVT_CORE_MPI_APPLICATION_CPP

#include "mpi.h"
#include "unistd.h"
#include "iostream"
#include <gvt/core/mpi/application.h>

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
