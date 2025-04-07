from pathfinding.core.grid import Grid
from pathfinding.finder.a_star import AStarFinder

def main():
    matrix = [
    [1, 1, 2, 0, 0, 0],
    [1, 1, 1, 1, 1, 0],
    [0, 1, 0, 0, 0, 0],
    [0, 0, 0, 1, 1, 1],
    [0, 0, 0, 0, 0, 0],
    [0, 0, 0, 0, 0, 0],
    ]  # 空地是>=1的, 空地是0,
    grid = Grid(matrix=matrix)
    start = grid.node(0, 0)
    end = grid.node(2, 0)
    finder = AStarFinder()
    path, runs = finder.find_path(start, end, grid)
    print('Path found with %d runs' % runs)
    print(path)

if __name__ == "__main__":
    main()
