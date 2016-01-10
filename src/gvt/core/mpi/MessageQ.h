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
