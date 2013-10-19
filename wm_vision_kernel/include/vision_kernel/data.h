/*
 * Copyright 2012-2013 Walking Machine
 *
 * Project: Walking Machine Sara robot 2012-2013
 * Package: wm_vision_kernel
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

/**
 * Generic data with runtime type validation
 */
class Data {
 public:
  /**
   * Constructor
   */
  Data() {
    data_type_ = &typeid(void);
  }

  /**
   * Get data with validation
   * @return Data if it's the good type else nullptr
   */
  template<class type>
  std::shared_ptr<type> DataWithValidation() {
    if (*data_type_ == typeid(type)) {
      return std::static_pointer_cast<type>(data_);
    } else {
      throw TypeValidationException();
      return nullptr;
    }
  }

  /**
   * Get data (warning: no type validation)
   * @return Casted data
   */
  template<class type>
  std::shared_ptr<type> data() {
    return std::static_pointer_cast<type>(data_);
  }

  /**
   * Valid runtime data type
   * @param data_type Data type to compare
   * @return True if it's the same runtime type
   */
  bool DataTypeEqual(std::type_info data_type) {
    return *data_type_ == data_type;
  }

  /**
   * Get runtime data type
   * @return Pointer to data type
   */
  const std::type_info* data_type() {
    return data_type_;
  }

  /**
   * Set data
   * @param data
   */
  template<class type>
  void set_data(std::shared_ptr<type> data) {
    data_ = data;
    data_type_ = &typeid(type);
  }

  /**
   * Type validation exception
   */
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
