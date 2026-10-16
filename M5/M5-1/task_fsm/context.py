class TaskContext:
    def __init__(self, task_id, deadline=None):
        self.task_id = task_id
        self.deadline = deadline
        self.data = {}
        self.retry_budget = {}
        self.degraded = False
        self.metadata = {}