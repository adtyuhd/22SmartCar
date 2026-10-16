from concurrent.futures import ThreadPoolExecutor, as_completed


class ParallelExecutor:
    def __init__(self, name, branches, fail_policy="all_must_succeed"):
        self.name = name
        self.branches = branches
        self.fail_policy = fail_policy
        self.params = {}
        self._cancelled = False

    def execute(self):
        if self._cancelled:
            return "SKIPPED"

        with ThreadPoolExecutor(max_workers=len(self.branches)) as executor:
            futures = {
                executor.submit(branch.execute): index
                for index, branch in enumerate(self.branches)
            }

            results = [None] * len(self.branches)

            for future in as_completed(futures):
                index = futures[future]
                result = future.result()
                results[index] = result

                if (
                    result == "FAILED"
                    and self.fail_policy == "any_fail"
                ):
                    for i, branch in enumerate(self.branches):
                        if i != index:
                            branch.cancel()

                    return "FAILED"

            if all(result == "SUCCESS" for result in results):
                return "SUCCESS"

            return "FAILED"

    def cancel(self):
        self._cancelled = True

        for branch in self.branches:
            branch.cancel()

    def reset_cancel(self):
        self._cancelled = False

        for branch in self.branches:
            branch.reset_cancel()