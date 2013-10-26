/*
 * vision_errors.h
 *
 *  Created on: Oct 24, 2013
 *      Author: mamorin
 */

#ifndef VISION_ERRORS_H_
#define VISION_ERRORS_H_

namespace vision_errors {
   enum Errors {
     NoError = 0,

     // Callback Flow
     InvalidTaskObjectError,
     InvalidActionName,
     FlowCreationError,
     CatalogNotLoaded,
     FlowAlreadyRunning,
     NoFlowRunning,

     FirstError = NoError,
     LastError = NoFlowRunning,
   };
}


#endif /* VISION_ERRORS_H_ */
