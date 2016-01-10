#include <mpi.h>
#include <pthread.h>
#include <unistd.h>
#include <iostream>
#include "Application.h"
#include <gvt/core/mpi/Message.h>
#include <gvt/core/mpi/MessageQ.h>

using namespace gvt::core::mpi;

void *
MessageManager::messageThread(void *p)
{
	MessageManager *theMessageManager = (MessageManager *)p;
  Application *theApplication = Application::GetApplication();

  MPI_Init(theApplication->GetPArgC(), theApplication->GetPArgV());
  MPI_Comm_rank(MPI_COMM_WORLD, &theMessageManager->mpi_rank);
  MPI_Comm_size(MPI_COMM_WORLD, &theMessageManager->mpi_size);

  pthread_mutex_lock(&theMessageManager->lock);

	theMessageManager->wait = 1;
	// bds put signal here to signal the main thread.
	pthread_cond_signal(&theMessageManager->cond);
	while (theMessageManager->wait == 1)
		pthread_cond_wait(&theMessageManager->cond, &theMessageManager->lock);

	theMessageManager->wait = 3;
	pthread_cond_signal(&theMessageManager->cond);
	pthread_mutex_unlock(&theMessageManager->lock);

  Message *message = new Message();
  while (! theApplication->IsDoneSet())
  {
    if (message->IsReady())
    {
      message->Receive();

			// Handle collective operations in the MPI thread

			if (message->header.collective)
			{
				Work *w = theApplication->Deserialize(message);
				delete message;
				bool kill_me = w->Action();
				delete w;
				if (kill_me)
				{
					theApplication->Kill();
				}
			}
			else
				theApplication->GetIncomingMessageQueue()->Enqueue(message);

      if (! theApplication->IsDoneSet())
        message = new Message();
    }

    while (theApplication->GetOutgoingMessageQueue()->IsReady())
    {
      Message *m = theApplication->GetOutgoingMessageQueue()->Dequeue();
			if (m)
			{
				// Send it on
				m->Send();

				// If this is a collective, execute its work here in the message thread
			  if (m->header.collective)
				{
					Work *w = theApplication->Deserialize(m);
					bool kill_me = w->Action();

					if (m->blocking)
					{
						pthread_mutex_lock(&m->lock);
						pthread_cond_signal(&m->cond);
						pthread_mutex_unlock(&m->lock);
					}
					else
						delete m;

					if (kill_me)
					{
						theApplication->Kill();
					}
				}
			}
			else
				break;
    }
  }


  // Make sure outgoing queue is flushed to make sure any
  // quit message is propagated

  while ((message = theApplication->GetOutgoingMessageQueue()->Dequeue()) != NULL)
  {
		message->Send();
		delete message;
  }

	MPI_Barrier(MPI_COMM_WORLD);
  MPI_Finalize();
  pthread_exit(NULL);
}

MessageManager::MessageManager()
{
	pthread_mutex_init(&lock, NULL);
	pthread_cond_init(&cond, NULL);
	pthread_mutex_lock(&lock);
}

MessageManager::~MessageManager()
{
	pthread_mutex_unlock(&lock);
}

void
MessageManager::Wait()
{
	pthread_join(tid, NULL);
}

void
MessageManager::Initialize()
{
	// create messageThread and wait for it to tell you it started.
	if (pthread_create(&tid, NULL, messageThread, this))
  {
    std::cerr << "Failed to spawn MPI thread\n";
    exit(1);
  }

	wait = 0;
	while(wait == 0)
		pthread_cond_wait(&cond, &lock);
}

void
MessageManager::Start()
{
	wait = 2;
	pthread_cond_signal(&cond);
	while (wait == 2)
		pthread_cond_wait(&cond, &lock);
}

Message::Message(Work* w, bool collective, bool b)
{
	header.type        = w->GetType();

	blocking = b;
	if (blocking)
	{
		pthread_mutex_init(&lock, NULL);
		pthread_cond_init(&cond, NULL);
		pthread_mutex_lock(&lock);
	}

	header.collective = collective;
	header.broadcast_root = Application::GetApplication()->GetRank();
	header.destination = -1;

	w->Serialize(header.size, serialized);
}

Message::Message(Work* w, int destination)
{
	header.type        = w->GetType();

	blocking = false;

	header.broadcast_root = -1;
	header.collective = false;
	header.destination = destination;

	w->Serialize(header.size, serialized);
}

Message::Message()
{
	serialized = NULL;
	MPI_Irecv((unsigned char *)&header, sizeof(header), MPI_UNSIGNED_CHAR, MPI_ANY_SOURCE, Message::HEADER_TAG, MPI_COMM_WORLD, &request);
}


Message::~Message()
{
	if (serialized) free(serialized);
}

void
Message::Wait()
{
	if (! blocking)
		std::cerr << "Error... waiting on non-blocking message?\n";
	else
		pthread_cond_wait(&cond, &lock);
}

bool
Message::IsReady()
{
	int read_ready;
	MPI_Test(&request, &read_ready, &status);
	return read_ready != 0;
}

void
Message::Enqueue()
{
	Application::GetApplication()->GetOutgoingMessageQueue()->Enqueue(this);
}

void
Message::Receive()
{
	if (header.size)
	{
		serialized = (unsigned char *)malloc(header.size);
		MPI_Recv(serialized, header.size, MPI_UNSIGNED_CHAR, header.source, Message::BODY_TAG, MPI_COMM_WORLD, &status);
	}

	if (header.broadcast_root != -1)
		Send();
}

void
Message::Send()
{
	header.source = Application::GetApplication()->GetRank();

	if (header.broadcast_root != -1)
	{
		int d = ((Application::GetApplication()->GetSize() + Application::GetApplication()->GetRank()) - header.broadcast_root) % Application::GetApplication()->GetSize();

		int l = (2*d) + 1;
		if (l < Application::GetApplication()->GetSize())
		{
			header.destination = (header.broadcast_root + l) % Application::GetApplication()->GetSize();

			MPI_Send((unsigned char *)&header, sizeof(header), MPI_UNSIGNED_CHAR, header.destination, Message::HEADER_TAG, MPI_COMM_WORLD);
			if (header.size)
				MPI_Send(serialized, header.size, MPI_UNSIGNED_CHAR, header.destination, Message::BODY_TAG, MPI_COMM_WORLD);

			if ((l+1) < Application::GetApplication()->GetSize())
			{
				header.destination = (header.broadcast_root + l + 1) % Application::GetApplication()->GetSize();

				MPI_Send((unsigned char *)&header, sizeof(header), MPI_UNSIGNED_CHAR, header.destination, Message::HEADER_TAG, MPI_COMM_WORLD);
				if (header.size)
					MPI_Send(serialized, header.size, MPI_UNSIGNED_CHAR, header.destination, Message::BODY_TAG, MPI_COMM_WORLD);
			}
		}

	}
	else
	{
		MPI_Send((unsigned char *)&header, sizeof(header), MPI_UNSIGNED_CHAR, header.destination, Message::HEADER_TAG, MPI_COMM_WORLD);
		if (header.size)
			MPI_Send(serialized, header.size, MPI_UNSIGNED_CHAR, header.destination, Message::BODY_TAG, MPI_COMM_WORLD);
	}
}
