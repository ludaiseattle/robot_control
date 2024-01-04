class IAction(object):
    def __init__(self, actionName):
        """The interface of action"""
        self.name = actionName

    def check(self):
        """Status checking"""
        return True

    def execute(self):
        """Execute the action"""
        return True
