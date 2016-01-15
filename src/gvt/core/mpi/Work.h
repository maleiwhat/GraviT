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
// Work.h
//

#ifndef GVT_CORE_MPI_WORK_H
#define GVT_CORE_MPI_WORK_H

#include <iostream>

using namespace std;

namespace gvt {
namespace core {
namespace mpi {

// This needs to be in the class definition of the Work subclass object - it'll
// define the creator and the registererh

#define WORK_CLASS_HEADER(ClassName)                               \
public:                                                            \
ClassName() { type = ClassName::class_type; initialize(); }        \
static void Register() {                                           \
   ClassName::class_type                                           \
       = Application::GetApplication()->RegisterWork(Deserialize); \
}                                                                  \
private:                                                           \
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
