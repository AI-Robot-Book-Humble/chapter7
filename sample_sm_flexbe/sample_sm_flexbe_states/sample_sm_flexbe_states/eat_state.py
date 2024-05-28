from flexbe_core import EventState, Logger


class EatState(EventState):
    """

    <= done
    """

    def __init__(self):
        super().__init__(outcomes=["done"])

        self.counter = 0

    def execute(self, userdata):
        Logger.loginfo("食べてます！")
        return "done"
