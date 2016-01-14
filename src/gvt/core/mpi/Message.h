#ifndef GVT_CORE_MPI_MESSAGE_H
#define GVT_CORE_MPI_MESSAGE_H

#include <mpi.h>
#include <stdlib.h>
#include "gvt/core/mpi/Work.h"

namespace gvt {
namespace core {
namespace mpi {

class MessageManager
{
public:
	MessageManager();
	~MessageManager();

	void Wait();

	void Initialize();
	void Start();

  int GetSize() { return mpi_size; }
  int GetRank() { return mpi_rank; }

private:

	static void* messageThread(void *);

	pthread_mutex_t lock;
	pthread_cond_t  cond;
	pthread_t				tid;

	int 						wait;
	int 						mpi_rank;
	int 						mpi_size;
};

class Message
{
	friend class MessageManager;

public:
	static const int HEADER_TAG = 1;
	static const int BODY_TAG   = 2;

	// Point to point, fire-and-forget
	Message(Work* w, int destination);

	// One-to-all, may be collective (that is, may be run in MPI thread), may be blocking
	Message(Work* w, bool collective = false, bool blocking = false);

	Message();
	~Message();

	bool IsReady();

	void Enqueue();

	virtual void Send();
	virtual void Receive();

	int  GetType() { return header.type; }
	size_t  GetSize() { return header.size; }
	unsigned char *GetBytes() { return serialized; }

	// Wait for message to be handled by MPI thread
	// If the message implies a collective, it'll therefore
	// be waiting until the collective completes.
	// ONLY VALID IF BLOCKING

	void Wait(); 

protected:

	struct
	{
		int		 	broadcast_root; 
		int		 	type;
		int		 	destination;
		int 		source;						
		size_t 	size;
		bool		collective;
	} header;

	int id;
	int pending;

	bool blocking;
	pthread_mutex_t lock;
	pthread_cond_t	cond;

	unsigned char *serialized;
	MPI_Status status;
	MPI_Request request;
	
};

} //ns mpi
} //ns core
} //ns gvt

#endif /* GVT_CORE_MPI_MESSAGE_H */