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
     InvalidActionName,
     FlowCreationError,
     CatalogNotLoaded,

     FIRST_ERROR = NoError,
     LAST_ERROR = CatalogNotLoaded,
   };
}


#endif /* VISION_ERRORS_H_ */
