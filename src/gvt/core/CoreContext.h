/* =======================================================================================
   This file is released as part of GraviT - scalable, platform independent ray tracing
   tacc.github.io/GraviT

   Copyright 2013-2015 Texas Advanced Computing Center, The University of Texas at Austin
   All rights reserved.

   Licensed under the BSD 3-Clause License, (the "License"); you may not use this file
   except in compliance with the License.
   A copy of the License is included with this software in the file LICENSE.
   If your copy does not contain the License, you may obtain a copy of the License at:

       http://opensource.org/licenses/BSD-3-Clause

   Unless required by applicable law or agreed to in writing, software distributed under
   the License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY
   KIND, either express or implied.
   See the License for the specific language governing permissions and limitations under
   limitations under the License.

   GraviT is funded in part by the US National Science Foundation under awards ACI-1339863,
   ACI-1339881 and ACI-1339840
   ======================================================================================= */
#ifndef GVT_CORE_CONTEXT_H
#define GVT_CORE_CONTEXT_H

#include <gvt/core/Debug.h>

#include "gvt/core/DatabaseNode.h"
#include "gvt/core/Types.h"
#include <gvt/core/Database.h>

#ifndef MAX
#define MAX(a, b) ((a > b) ? (a) : (b))
#endif

#ifndef MIN
#define MIN(a, b) ((a < b) ? (a) : (b))
#endif

#define CONTEXT_LEAF_MARSH_SIZE 256

namespace gvt {
namespace core {
/// context base class for GraviT internal state
/** base class for GraviT internal state.
The context contains the object-store database and helper methods to create and
manage the internal state.
*/
class CoreContext {
public:
  virtual ~CoreContext();

  /// return the context singleton
  static CoreContext *instance();

  /// return the object-store database
  Database *database() {
    GVT_ASSERT(__database != nullptr, "The context seems to be uninitialized.");
    return __database;
  }

  /// return a handle to the root of the object-store hierarchy
  DBNodeH getRootNode() {
    GVT_ASSERT(__database != nullptr, "The context seems to be uninitialized.");
    return __rootNode;
  }

  /// return a handle to the node with matching UUID
  DBNodeH getNode(Uuid);

  /// create a node in the database
  /** \param name the node name
      \param val the node value
      \param parent the uuid of the parent node
      */
  DBNodeH createNode(String name, Variant val = Variant(String("")), Uuid parent = Uuid::null());

  /// create a node of the specified type in the database
  DBNodeH createNodeFromType(String);

  /// create a node of the specified type with the specifed parent uuid
  DBNodeH createNodeFromType(String, Uuid);

  /// create a node of the specified type with the specified name and parent
  /** \param type the type of node to create
      \param name the node name
      \param parent the uuid of the parent node
      */
  DBNodeH createNodeFromType(String type, String name, Uuid parent = Uuid::null());

  /**
   * Packs a node and its leaf children in a buffer
   * Byte structure:
   * <nodeParentName><nodeName><int variant type><value><#children><child0Name><int variant type><value><...>
   * Each node or leaf is packed in CONTEXT_LEAF_MARSH_SIZE max byte size,
   * i.e. <nodeName><int variant type><value> = CONTEXT_LEAF_MARSH_SIZE bytes
   */
	void marshNode(unsigned char *buffer, DBNodeH& node) {

		const char * parentName =
				__database->getItem(node.parentUUID())->value().toString().c_str();

		memcpy(buffer, parentName, strlen(parentName) + 1);
		buffer += strlen(parentName) + 1;

		DatabaseNode::marshLeaf(buffer, node.getNode());
		buffer += CONTEXT_LEAF_MARSH_SIZE;

		Vector<DBNodeH> children = node.getChildren();
		int nChildren = children.size();
		memcpy(buffer, &(nChildren), sizeof(int));
		buffer += sizeof(int);
		for (auto leaf : children) {
			DatabaseNode::marshLeaf(buffer, leaf.getNode());
			buffer += CONTEXT_LEAF_MARSH_SIZE;
		}
	}

  // check marsh
  DBNodeH unmarsh(unsigned char *buffer){

	  String parentName = std::string((char*) buffer);
	  buffer+=parentName.size()+1;

	  DatabaseNode * unmarshedParent = DatabaseNode::unmarshLeaf(buffer, Uuid());
	  DBNodeH unmarshedParentHandler = DBNodeH(unmarshedParent->UUID());
	  __database->setItem(unmarshedParent);

	  buffer+=CONTEXT_LEAF_MARSH_SIZE;

	  int nChildren =*(int*)buffer;
	  buffer+=sizeof(int);


	  for(int i=0; i < nChildren; i++){
		  DatabaseNode *unmarshedChild = DatabaseNode::unmarshLeaf(buffer, unmarshedParentHandler.UUID());
		  DBNodeH unmarshedChildHandler = DBNodeH(unmarshedChild->UUID());
		  __database->setItem(unmarshedChild);
		  buffer+=CONTEXT_LEAF_MARSH_SIZE;

	  }

	  return unmarshedParentHandler;

  }



protected:
  CoreContext();
  static CoreContext *__singleton;
  Database *__database = nullptr;
  DBNodeH __rootNode;
};
}
}

#endif // GVT_CORE_CONTEXT_H
