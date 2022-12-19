from a_star import AStar


def main():
    """start the assignment"""
    task1 = AStar(task=1)
    task1.compute()
    task1.print()

    task2 = AStar(task=2, distance="manhattan")
    task2.compute()
    task2.print()

    task3 = AStar(task=3)
    task3.compute()
    task3.print()

    task4 = AStar(task=4)
    task4.compute()
    task4.print()


if __name__ == "__main__":
    main()
