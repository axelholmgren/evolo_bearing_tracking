

class TimingNode(Node):
    """
    Timing node for comparing timestamps across multiple messages
    """

    def __init__(self):
        super().__init("timing_node")
        

        self.subscrition = self.create_subscription
