// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from package_with_interfaces:srv/ObjectGrab.idl
// generated code does not contain a copyright notice
#include "package_with_interfaces/srv/detail/object_grab__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"

// Include directives for member types
// Member `object`
#include "package_with_interfaces/msg/detail/object_information__functions.h"

bool
package_with_interfaces__srv__ObjectGrab_Request__init(package_with_interfaces__srv__ObjectGrab_Request * msg)
{
  if (!msg) {
    return false;
  }
  // object
  if (!package_with_interfaces__msg__ObjectInformation__init(&msg->object)) {
    package_with_interfaces__srv__ObjectGrab_Request__fini(msg);
    return false;
  }
  return true;
}

void
package_with_interfaces__srv__ObjectGrab_Request__fini(package_with_interfaces__srv__ObjectGrab_Request * msg)
{
  if (!msg) {
    return;
  }
  // object
  package_with_interfaces__msg__ObjectInformation__fini(&msg->object);
}

bool
package_with_interfaces__srv__ObjectGrab_Request__are_equal(const package_with_interfaces__srv__ObjectGrab_Request * lhs, const package_with_interfaces__srv__ObjectGrab_Request * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // object
  if (!package_with_interfaces__msg__ObjectInformation__are_equal(
      &(lhs->object), &(rhs->object)))
  {
    return false;
  }
  return true;
}

bool
package_with_interfaces__srv__ObjectGrab_Request__copy(
  const package_with_interfaces__srv__ObjectGrab_Request * input,
  package_with_interfaces__srv__ObjectGrab_Request * output)
{
  if (!input || !output) {
    return false;
  }
  // object
  if (!package_with_interfaces__msg__ObjectInformation__copy(
      &(input->object), &(output->object)))
  {
    return false;
  }
  return true;
}

package_with_interfaces__srv__ObjectGrab_Request *
package_with_interfaces__srv__ObjectGrab_Request__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  package_with_interfaces__srv__ObjectGrab_Request * msg = (package_with_interfaces__srv__ObjectGrab_Request *)allocator.allocate(sizeof(package_with_interfaces__srv__ObjectGrab_Request), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(package_with_interfaces__srv__ObjectGrab_Request));
  bool success = package_with_interfaces__srv__ObjectGrab_Request__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
package_with_interfaces__srv__ObjectGrab_Request__destroy(package_with_interfaces__srv__ObjectGrab_Request * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    package_with_interfaces__srv__ObjectGrab_Request__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
package_with_interfaces__srv__ObjectGrab_Request__Sequence__init(package_with_interfaces__srv__ObjectGrab_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  package_with_interfaces__srv__ObjectGrab_Request * data = NULL;

  if (size) {
    data = (package_with_interfaces__srv__ObjectGrab_Request *)allocator.zero_allocate(size, sizeof(package_with_interfaces__srv__ObjectGrab_Request), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = package_with_interfaces__srv__ObjectGrab_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        package_with_interfaces__srv__ObjectGrab_Request__fini(&data[i - 1]);
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
package_with_interfaces__srv__ObjectGrab_Request__Sequence__fini(package_with_interfaces__srv__ObjectGrab_Request__Sequence * array)
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
      package_with_interfaces__srv__ObjectGrab_Request__fini(&array->data[i]);
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

package_with_interfaces__srv__ObjectGrab_Request__Sequence *
package_with_interfaces__srv__ObjectGrab_Request__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  package_with_interfaces__srv__ObjectGrab_Request__Sequence * array = (package_with_interfaces__srv__ObjectGrab_Request__Sequence *)allocator.allocate(sizeof(package_with_interfaces__srv__ObjectGrab_Request__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = package_with_interfaces__srv__ObjectGrab_Request__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
package_with_interfaces__srv__ObjectGrab_Request__Sequence__destroy(package_with_interfaces__srv__ObjectGrab_Request__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    package_with_interfaces__srv__ObjectGrab_Request__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
package_with_interfaces__srv__ObjectGrab_Request__Sequence__are_equal(const package_with_interfaces__srv__ObjectGrab_Request__Sequence * lhs, const package_with_interfaces__srv__ObjectGrab_Request__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!package_with_interfaces__srv__ObjectGrab_Request__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
package_with_interfaces__srv__ObjectGrab_Request__Sequence__copy(
  const package_with_interfaces__srv__ObjectGrab_Request__Sequence * input,
  package_with_interfaces__srv__ObjectGrab_Request__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(package_with_interfaces__srv__ObjectGrab_Request);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    package_with_interfaces__srv__ObjectGrab_Request * data =
      (package_with_interfaces__srv__ObjectGrab_Request *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!package_with_interfaces__srv__ObjectGrab_Request__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          package_with_interfaces__srv__ObjectGrab_Request__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!package_with_interfaces__srv__ObjectGrab_Request__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


bool
package_with_interfaces__srv__ObjectGrab_Response__init(package_with_interfaces__srv__ObjectGrab_Response * msg)
{
  if (!msg) {
    return false;
  }
  // success
  return true;
}

void
package_with_interfaces__srv__ObjectGrab_Response__fini(package_with_interfaces__srv__ObjectGrab_Response * msg)
{
  if (!msg) {
    return;
  }
  // success
}

bool
package_with_interfaces__srv__ObjectGrab_Response__are_equal(const package_with_interfaces__srv__ObjectGrab_Response * lhs, const package_with_interfaces__srv__ObjectGrab_Response * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // success
  if (lhs->success != rhs->success) {
    return false;
  }
  return true;
}

bool
package_with_interfaces__srv__ObjectGrab_Response__copy(
  const package_with_interfaces__srv__ObjectGrab_Response * input,
  package_with_interfaces__srv__ObjectGrab_Response * output)
{
  if (!input || !output) {
    return false;
  }
  // success
  output->success = input->success;
  return true;
}

package_with_interfaces__srv__ObjectGrab_Response *
package_with_interfaces__srv__ObjectGrab_Response__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  package_with_interfaces__srv__ObjectGrab_Response * msg = (package_with_interfaces__srv__ObjectGrab_Response *)allocator.allocate(sizeof(package_with_interfaces__srv__ObjectGrab_Response), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(package_with_interfaces__srv__ObjectGrab_Response));
  bool success = package_with_interfaces__srv__ObjectGrab_Response__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
package_with_interfaces__srv__ObjectGrab_Response__destroy(package_with_interfaces__srv__ObjectGrab_Response * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    package_with_interfaces__srv__ObjectGrab_Response__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
package_with_interfaces__srv__ObjectGrab_Response__Sequence__init(package_with_interfaces__srv__ObjectGrab_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  package_with_interfaces__srv__ObjectGrab_Response * data = NULL;

  if (size) {
    data = (package_with_interfaces__srv__ObjectGrab_Response *)allocator.zero_allocate(size, sizeof(package_with_interfaces__srv__ObjectGrab_Response), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = package_with_interfaces__srv__ObjectGrab_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        package_with_interfaces__srv__ObjectGrab_Response__fini(&data[i - 1]);
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
package_with_interfaces__srv__ObjectGrab_Response__Sequence__fini(package_with_interfaces__srv__ObjectGrab_Response__Sequence * array)
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
      package_with_interfaces__srv__ObjectGrab_Response__fini(&array->data[i]);
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

package_with_interfaces__srv__ObjectGrab_Response__Sequence *
package_with_interfaces__srv__ObjectGrab_Response__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  package_with_interfaces__srv__ObjectGrab_Response__Sequence * array = (package_with_interfaces__srv__ObjectGrab_Response__Sequence *)allocator.allocate(sizeof(package_with_interfaces__srv__ObjectGrab_Response__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = package_with_interfaces__srv__ObjectGrab_Response__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
package_with_interfaces__srv__ObjectGrab_Response__Sequence__destroy(package_with_interfaces__srv__ObjectGrab_Response__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    package_with_interfaces__srv__ObjectGrab_Response__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
package_with_interfaces__srv__ObjectGrab_Response__Sequence__are_equal(const package_with_interfaces__srv__ObjectGrab_Response__Sequence * lhs, const package_with_interfaces__srv__ObjectGrab_Response__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!package_with_interfaces__srv__ObjectGrab_Response__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
package_with_interfaces__srv__ObjectGrab_Response__Sequence__copy(
  const package_with_interfaces__srv__ObjectGrab_Response__Sequence * input,
  package_with_interfaces__srv__ObjectGrab_Response__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(package_with_interfaces__srv__ObjectGrab_Response);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    package_with_interfaces__srv__ObjectGrab_Response * data =
      (package_with_interfaces__srv__ObjectGrab_Response *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!package_with_interfaces__srv__ObjectGrab_Response__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          package_with_interfaces__srv__ObjectGrab_Response__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!package_with_interfaces__srv__ObjectGrab_Response__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
