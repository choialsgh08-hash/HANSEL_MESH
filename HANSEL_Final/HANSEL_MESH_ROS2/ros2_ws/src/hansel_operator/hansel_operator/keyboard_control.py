"""ROS/Qt-independent hold-to-drive session and short input-source lease."""
LIVE_KEYS = {
    'w':'forward', 's':'backward', 'a':'left', 'd':'right',
    'q':'forward_left', 'e':'forward_right', 'z':'backward_left', 'c':'backward_right',
    'x':'stop', ' ':'stop',
}


class KeyboardSession:
    def __init__(self, publish):
        self.publish = publish
        self.active = False
        self.command = 'stop'
        self.held_key = None

    def focus(self, active):
        active = bool(active)
        if active != self.active:
            self.active = active
            self.command, self.held_key = 'stop', None
            self.publish('stop')

    def key(self, key):
        if not self.active or key not in LIVE_KEYS:
            return False
        self.command, self.held_key = LIVE_KEYS[key], key
        self.publish(self.command)
        return True

    def release(self, key):
        if self.active and key == self.held_key:
            self.command, self.held_key = 'stop', None
            self.publish('stop')

    def tick(self):
        if self.active:
            self.publish(self.command)


class InputLease:
    def __init__(self, timeout=0.4):
        self.timeout = timeout
        self.owner = None
        self.expires = 0.0

    def accept(self, source, now):
        if now >= self.expires:
            self.owner = None
        if self.owner is not None and source != self.owner:
            return False
        if source.startswith('rqt:'):
            self.owner, self.expires = source, now + self.timeout
        return True
