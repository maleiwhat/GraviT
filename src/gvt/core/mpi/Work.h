#ifndef GVT_CORE_MPI_WORK_H
#define GVT_CORE_MPI_WORK_H

#include <iostream>

using namespace std;

namespace gvt {
namespace core {
namespace mpi {

// This needs to be in the class definition of the Work subclass object - it'll
// define the creator and the registererh

#define WORK_CLASS_HEADER(ClassName) 																						 					\
public:																																					 					\
ClassName() { type = ClassName::class_type; initialize(); }																\
static void Register() {																																	\
	ClassName::class_type = Application::GetApplication()->RegisterWork(Deserialize);				\
}																																												 	\
private:																																				 					\
static int class_type;

// This needs to be in a source file somewhere to

#define WORK_CLASS(ClassName) int ClassName::class_type = 0;

class Work
{
public:
	Work() {}
	~Work() {}


	static Work *Deserialize(size_t size, unsigned char * body);

	virtual void initialize() {};
	virtual void Serialize(size_t& size, unsigned char *& serialized);
	virtual bool Action() {return true;};
	virtual void Send(int destination);
	virtual void Broadcast(bool collective, bool block);

	int GetType() { return type; }

protected:
	int type;
};

} //ns mpi
} //ns core
} //ns gvt

#endif /* GVT_CORE_MPI_WORK_H */
