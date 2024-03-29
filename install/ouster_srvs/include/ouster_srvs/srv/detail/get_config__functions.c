// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from ouster_srvs:srv/GetConfig.idl
// generated code does not contain a copyright notice
#include "ouster_srvs/srv/detail/get_config__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"

bool
ouster_srvs__srv__GetConfig_Request__init(ouster_srvs__srv__GetConfig_Request * msg)
{
  if (!msg) {
    return false;
  }
  // structure_needs_at_least_one_member
  return true;
}

void
ouster_srvs__srv__GetConfig_Request__fini(ouster_srvs__srv__GetConfig_Request * msg)
{
  if (!msg) {
    return;
  }
  // structure_needs_at_least_one_member
}

bool
ouster_srvs__srv__GetConfig_Request__are_equal(const ouster_srvs__srv__GetConfig_Request * lhs, const ouster_srvs__srv__GetConfig_Request * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // structure_needs_at_least_one_member
  if (lhs->structure_needs_at_least_one_member != rhs->structure_needs_at_least_one_member) {
    return false;
  }
  return true;
}

bool
ouster_srvs__srv__GetConfig_Request__copy(
  const ouster_srvs__srv__GetConfig_Request * input,
  ouster_srvs__srv__GetConfig_Request * output)
{
  if (!input || !output) {
    return false;
  }
  // structure_needs_at_least_one_member
  output->structure_needs_at_least_one_member = input->structure_needs_at_least_one_member;
  return true;
}

ouster_srvs__srv__GetConfig_Request *
ouster_srvs__srv__GetConfig_Request__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ouster_srvs__srv__GetConfig_Request * msg = (ouster_srvs__srv__GetConfig_Request *)allocator.allocate(sizeof(ouster_srvs__srv__GetConfig_Request), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(ouster_srvs__srv__GetConfig_Request));
  bool success = ouster_srvs__srv__GetConfig_Request__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
ouster_srvs__srv__GetConfig_Request__destroy(ouster_srvs__srv__GetConfig_Request * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    ouster_srvs__srv__GetConfig_Request__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
ouster_srvs__srv__GetConfig_Request__Sequence__init(ouster_srvs__srv__GetConfig_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ouster_srvs__srv__GetConfig_Request * data = NULL;

  if (size) {
    data = (ouster_srvs__srv__GetConfig_Request *)allocator.zero_allocate(size, sizeof(ouster_srvs__srv__GetConfig_Request), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = ouster_srvs__srv__GetConfig_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        ouster_srvs__srv__GetConfig_Request__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
ouster_srvs__srv__GetConfig_Request__Sequence__fini(ouster_srvs__srv__GetConfig_Request__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      ouster_srvs__srv__GetConfig_Request__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

ouster_srvs__srv__GetConfig_Request__Sequence *
ouster_srvs__srv__GetConfig_Request__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ouster_srvs__srv__GetConfig_Request__Sequence * array = (ouster_srvs__srv__GetConfig_Request__Sequence *)allocator.allocate(sizeof(ouster_srvs__srv__GetConfig_Request__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = ouster_srvs__srv__GetConfig_Request__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
ouster_srvs__srv__GetConfig_Request__Sequence__destroy(ouster_srvs__srv__GetConfig_Request__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    ouster_srvs__srv__GetConfig_Request__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
ouster_srvs__srv__GetConfig_Request__Sequence__are_equal(const ouster_srvs__srv__GetConfig_Request__Sequence * lhs, const ouster_srvs__srv__GetConfig_Request__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!ouster_srvs__srv__GetConfig_Request__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
ouster_srvs__srv__GetConfig_Request__Sequence__copy(
  const ouster_srvs__srv__GetConfig_Request__Sequence * input,
  ouster_srvs__srv__GetConfig_Request__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(ouster_srvs__srv__GetConfig_Request);
    ouster_srvs__srv__GetConfig_Request * data =
      (ouster_srvs__srv__GetConfig_Request *)realloc(output->data, allocation_size);
    if (!data) {
      return false;
    }
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!ouster_srvs__srv__GetConfig_Request__init(&data[i])) {
        /* free currently allocated and return false */
        for (; i-- > output->capacity; ) {
          ouster_srvs__srv__GetConfig_Request__fini(&data[i]);
        }
        free(data);
        return false;
      }
    }
    output->data = data;
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!ouster_srvs__srv__GetConfig_Request__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `config`
#include "rosidl_runtime_c/string_functions.h"

bool
ouster_srvs__srv__GetConfig_Response__init(ouster_srvs__srv__GetConfig_Response * msg)
{
  if (!msg) {
    return false;
  }
  // config
  if (!rosidl_runtime_c__String__init(&msg->config)) {
    ouster_srvs__srv__GetConfig_Response__fini(msg);
    return false;
  }
  return true;
}

void
ouster_srvs__srv__GetConfig_Response__fini(ouster_srvs__srv__GetConfig_Response * msg)
{
  if (!msg) {
    return;
  }
  // config
  rosidl_runtime_c__String__fini(&msg->config);
}

bool
ouster_srvs__srv__GetConfig_Response__are_equal(const ouster_srvs__srv__GetConfig_Response * lhs, const ouster_srvs__srv__GetConfig_Response * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // config
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->config), &(rhs->config)))
  {
    return false;
  }
  return true;
}

bool
ouster_srvs__srv__GetConfig_Response__copy(
  const ouster_srvs__srv__GetConfig_Response * input,
  ouster_srvs__srv__GetConfig_Response * output)
{
  if (!input || !output) {
    return false;
  }
  // config
  if (!rosidl_runtime_c__String__copy(
      &(input->config), &(output->config)))
  {
    return false;
  }
  return true;
}

ouster_srvs__srv__GetConfig_Response *
ouster_srvs__srv__GetConfig_Response__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ouster_srvs__srv__GetConfig_Response * msg = (ouster_srvs__srv__GetConfig_Response *)allocator.allocate(sizeof(ouster_srvs__srv__GetConfig_Response), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(ouster_srvs__srv__GetConfig_Response));
  bool success = ouster_srvs__srv__GetConfig_Response__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
ouster_srvs__srv__GetConfig_Response__destroy(ouster_srvs__srv__GetConfig_Response * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    ouster_srvs__srv__GetConfig_Response__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
ouster_srvs__srv__GetConfig_Response__Sequence__init(ouster_srvs__srv__GetConfig_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ouster_srvs__srv__GetConfig_Response * data = NULL;

  if (size) {
    data = (ouster_srvs__srv__GetConfig_Response *)allocator.zero_allocate(size, sizeof(ouster_srvs__srv__GetConfig_Response), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = ouster_srvs__srv__GetConfig_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        ouster_srvs__srv__GetConfig_Response__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
ouster_srvs__srv__GetConfig_Response__Sequence__fini(ouster_srvs__srv__GetConfig_Response__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      ouster_srvs__srv__GetConfig_Response__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

ouster_srvs__srv__GetConfig_Response__Sequence *
ouster_srvs__srv__GetConfig_Response__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ouster_srvs__srv__GetConfig_Response__Sequence * array = (ouster_srvs__srv__GetConfig_Response__Sequence *)allocator.allocate(sizeof(ouster_srvs__srv__GetConfig_Response__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = ouster_srvs__srv__GetConfig_Response__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
ouster_srvs__srv__GetConfig_Response__Sequence__destroy(ouster_srvs__srv__GetConfig_Response__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    ouster_srvs__srv__GetConfig_Response__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
ouster_srvs__srv__GetConfig_Response__Sequence__are_equal(const ouster_srvs__srv__GetConfig_Response__Sequence * lhs, const ouster_srvs__srv__GetConfig_Response__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!ouster_srvs__srv__GetConfig_Response__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
ouster_srvs__srv__GetConfig_Response__Sequence__copy(
  const ouster_srvs__srv__GetConfig_Response__Sequence * input,
  ouster_srvs__srv__GetConfig_Response__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(ouster_srvs__srv__GetConfig_Response);
    ouster_srvs__srv__GetConfig_Response * data =
      (ouster_srvs__srv__GetConfig_Response *)realloc(output->data, allocation_size);
    if (!data) {
      return false;
    }
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!ouster_srvs__srv__GetConfig_Response__init(&data[i])) {
        /* free currently allocated and return false */
        for (; i-- > output->capacity; ) {
          ouster_srvs__srv__GetConfig_Response__fini(&data[i]);
        }
        free(data);
        return false;
      }
    }
    output->data = data;
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!ouster_srvs__srv__GetConfig_Response__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
