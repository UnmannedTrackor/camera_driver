//=============================================================================
// Copyright © 2017 FLIR Integrated Imaging Solutions, Inc. All Rights Reserved.
//
// This software is the confidential and proprietary information of FLIR
// Integrated Imaging Solutions, Inc. ("Confidential Information"). You
// shall not disclose such Confidential Information and shall use it only in
// accordance with the terms of the license agreement you entered into
// with FLIR Integrated Imaging Solutions, Inc. (FLIR).
//
// FLIR MAKES NO REPRESENTATIONS OR WARRANTIES ABOUT THE SUITABILITY OF THE
// SOFTWARE, EITHER EXPRESSED OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR
// PURPOSE, OR NON-INFRINGEMENT. FLIR SHALL NOT BE LIABLE FOR ANY DAMAGES
// SUFFERED BY LICENSEE AS A RESULT OF USING, MODIFYING OR DISTRIBUTING
// THIS SOFTWARE OR ITS DERIVATIVES.
//=============================================================================
/*!
\file
\brief    Definition of RegisterNode
\ingroup Spinnaker_GenApi_PublicImpl
*/

#ifndef SPINNAKER_GENAPI_REGISTERNODE_H
#define SPINNAKER_GENAPI_REGISTERNODE_H

#include "SpinnakerPlatform.h"
#include "Types.h"
#include "Base.h"
#include "GCString.h"
#include "ISelector.h"
#include "INode.h"
#include "ValueNode.h"
#include "IRegister.h"

#pragma warning ( push )
#pragma warning( disable : 4250 ) // C4250 - 'class1' : inherits 'class2::member' via dominance
#pragma warning ( disable : 4251 ) // XXX needs to have dll-interface to be used by clients of class YYY
#pragma warning( disable: 4275 ) // non dll-interface structXXX used as base

namespace Spinnaker
{
	namespace GenApi
	{
		/**
		\brief Interface for string properties
		\ingroup Spinnaker_GenApi_PublicInterface
		*/
        class SPINNAKER_API RegisterNode : virtual public IRegister, virtual public ValueNode
		{
		public:
            struct NodeImpl;
			RegisterNode();

			RegisterNode(std::shared_ptr<Node::NodeImpl> pRegister);

			virtual ~RegisterNode();

			//! Set the register's contents
			/*!
			\param pBuffer The buffer containing the data to set
			\param Length The number of bytes in pBuffer
			\param Verify Enables AccessMode and Range verification (default = true)
			*/
			virtual void Set(const uint8_t *pBuffer, int64_t Length, bool Verify = true);

			//! Fills a buffer with the register's contents
			/*!
			\param pBuffer The buffer receiving the data to read
			\param Length The number of bytes to retrieve
			\param Verify Enables Range verification (default = false). The AccessMode is always checked
			\param IgnoreCache If true the value is read ignoring any caches (default = false)
			\return The value read
			*/
			virtual void Get(uint8_t *pBuffer, int64_t Length, bool Verify = false, bool IgnoreCache = false);

			//! Retrieves the Length of the register [Bytes]
			virtual int64_t GetLength();

			//! Retrieves the Address of the register
			virtual int64_t GetAddress();

			//! overload SetReference for Register
			virtual void SetReference(INode* pBase);

		private:

			std::shared_ptr<Node::NodeImpl> m_pRegister;

		};

		typedef RegisterNode CRegisterRef;
	}
}

#endif // SPINNAKER_GENAPI_REGISTERNODE_H