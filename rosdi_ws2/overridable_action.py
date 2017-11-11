import time

class OverridableAction(object):

    def __init__(self, node, topic_name, topic_type, callback_action, timeout, override_action):
        self._node = node
        self._topic_name = topic_name
        self._topic_type = topic_type
        self._callback_action = callback_action
        self._timeout = timeout
        self._override_action = override_action

        self._subscription = node.create_subscription(
            topic_type,
            topic_name,
            self._topic_callback)

        self._reset_last_update()

    def verify_override(self):
        if not self._override_called and \
           (self._last_update < int(round(time.time() * 1000)) - self._timeout):
            self._override_action()
            self._override_called = True

    def _topic_callback(self, msg):
        self._reset_last_update()
        self._callback_action(msg)

    def _reset_last_update(self):
        self._last_update = int(round(time.time() * 1000))
        self._override_called = False
