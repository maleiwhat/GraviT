#include <unistd.h>
#include "iostream"

#include <gvt/core/mpi/MessageQ.h>
#include <gvt/core/mpi/Message.h>
#include <gvt/core/mpi/Application.h>

using namespace std;
using namespace gvt::core::mpi;

void
MessageQ::Enqueue(Message *w)
{
	pthread_mutex_lock(&lock);
	workq.push(w);
	pthread_cond_signal(&signal);
	pthread_mutex_unlock(&lock);
}

Message *
MessageQ::Dequeue()
{
	pthread_mutex_lock(&lock);

	while (workq.empty() && running)
		pthread_cond_wait(&signal, &lock);

	Message *r;
	if (workq.empty())
		r = NULL;
	else
	{
		r = workq.front();
		workq.pop();
	}

	pthread_mutex_unlock(&lock);
	return r;
}

int
MessageQ::IsReady()
{
	pthread_mutex_lock(&lock);
	int t = (workq.empty() && running) ? 0 : 1;
	pthread_mutex_unlock(&lock);
	return t;
}

void
MessageQ::Kill()
{
	running = false;
	pthread_cond_broadcast(&signal);
}
