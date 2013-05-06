/*
 * Copyright 2012-2013 Walking Machine
 *
 * Project: Walking Machine Sara robot 2012-2013
 * Package: wm_vision
 * Node: vision_kernel
 *
 * Creation date: 12/06/2012
 *
 * Programmer: Keaven Martin
 *
 * Description: Generic data
 *
 */

#ifndef WM_VISION_INCLUDE_VISION_KERNEL_DATA_H_
#define WM_VISION_INCLUDE_VISION_KERNEL_DATA_H_

#include <memory>
#include <typeinfo>

class Data {
 public:
  Data() {
    data_type_ = &typeid(void);
  }

  template<class type>
  std::shared_ptr<type> DataWithValidation() {
    if (*data_type_ == typeid(type)) {
      return std::static_pointer_cast<type>(data_);
    } else {
      throw TypeValidationException();
      return nullptr;
    }
  }

  template<class type>
  std::shared_ptr<type> data() {
    return std::static_pointer_cast<type>(data_);
  }

  bool DataTypeEqual(std::type_info data_type) {
    return *data_type_ == data_type;
  }

  const std::type_info* data_type() {
    return data_type_;
  }

  template<class type>
  void set_data(std::shared_ptr<type> data) {
    data_ = data;
    data_type_ = &typeid(type);
  }

  class TypeValidationException : public std::exception {
    virtual const char* what() const throw() {
      return "Invalid type";
    }
  };

 private:
  std::shared_ptr<void> data_;
  const std::type_info *data_type_;
};

#endif  // WM_VISION_INCLUDE_VISION_KERNEL_DATA_H_
