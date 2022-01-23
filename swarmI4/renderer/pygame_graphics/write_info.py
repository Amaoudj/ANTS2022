
class Info:
    def __init__(self,text, is_title:bool=False):
        """
        class for storing info
        """
        self.is_title = is_title # True for title , False for normal text
        self.text =text