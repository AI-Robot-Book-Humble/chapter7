from flexbe_core import EventState, Logger


class SearchState(EventState):
    """

    <= succeeded
    <= finished
    """

    def __init__(self):
        super().__init__(outcomes=["succeeded", "finished"])

        self.counter = 0

    def execute(self, userdata):
        Logger.loginfo("探索中です")

        if self.counter < 3:
            Logger.loginfo("スイーツを見つけました！")
            self.counter += 1
            return "succeeded"

        else:
            Logger.loginfo("お腹いっぱいです・・・")
            return "finished"
