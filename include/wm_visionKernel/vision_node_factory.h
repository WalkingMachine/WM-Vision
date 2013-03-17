/*
 * Copyright 2012-2013 Walking Machine
 *
 * Project: Walking Machine Sara robot 2012-2013
 * Package: wm_vision
 * Node: wm_visionKernel
 *
 * Creation date: 21/02/2013
 *
 * Programmer: Keaven Martin
 *
 * Description:
 *
 */

#ifndef WM_VISION_INCLUDE_WM_VISIONKERNEL_VISION_NODE_FACTORY_H_
#define WM_VISION_INCLUDE_WM_VISIONKERNEL_VISION_NODE_FACTORY_H_

  #include <boost/thread.hpp>
  #include <string>
  #include <map>

  #include "../include/wm_visionKernel/vision_node.h"

  class VisionNode;

  // Factory Macro
  #define REGISTER_DEC_TYPE(NAME) \
    static VisionNodeFactoryRegister<NAME> reg

  #define REGISTER_DEF_TYPE(NAME) \
    VisionNodeFactoryRegister<NAME> NAME::reg(#NAME)

  // Factory
  template<typename T>
    VisionNode *createT() {
      return new T;
  }

  struct VisionNodeFactory {
    public:
      typedef std::map<std::string, VisionNode*(*)()> MapType;

      static VisionNode *CreateInstance(std::string const& s) {
        MapType::iterator it = map()->find(s);

        if (it == map()->end())
          return 0;

        return it->second();
      }

    protected:
      static MapType *map() {
        // never delete'ed. (exist until program termination)
        // because we can't guarantee correct destruction order
        if (!map_)
          map_ = new MapType;

        return map_;
      }

    private:
      static MapType *map_;
  };

  template<typename T>
  struct VisionNodeFactoryRegister: VisionNodeFactory {
    explicit VisionNodeFactoryRegister(std::string const& s) {
      map()->insert(std::make_pair(s, &createT<T>));
    }
  };

#endif  // WM_VISION_INCLUDE_WM_VISIONKERNEL_VISION_NODE_FACTORY_H_