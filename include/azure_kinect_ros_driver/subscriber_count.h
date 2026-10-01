// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#ifndef SUBSCRIBER_COUNT_H
#define SUBSCRIBER_COUNT_H

// System headers
//
#include <cstddef>

namespace azure_kinect_ros_driver
{
/**
 * @brief Number of subscribers of a publisher that may not have been created.
 *
 * Subscribers in the same process, which get the messages through intra-process communication,
 * are not counted as subscriptions of the publisher, so they are added.
 *
 * @tparam PublisherPtr A pointer to a lifecycle publisher.
 * @param publisher The publisher, or nullptr for a stream that is disabled.
 * @return The number of subscribers, zero if the publisher does not exist or is not active.
 */
template <typename PublisherPtr>
size_t subscriberCount(const PublisherPtr& publisher)
{
  if (!publisher || !publisher->is_activated())
  {
    return 0;
  }
  return publisher->get_subscription_count() + publisher->get_intra_process_subscription_count();
}
}  // namespace azure_kinect_ros_driver

#endif  // SUBSCRIBER_COUNT_H
