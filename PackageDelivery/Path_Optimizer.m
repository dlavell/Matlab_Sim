obstacles = [
    0  0  0  0  0    0  0  0  0  0;
    0  0  0  0  0    0  0  0  0  0;
    0  0  0  0  0    0  1  1  0  0;
    0  0  0  1  0    0  0  0  0  0;
    0  0  0  0  0.0  0  0  0  0  0;
    0  0  0  0  0    0  0  0  0  0;
    0  0  0  0  1    1  0  0  0  0;
    0  0  0  0  1    1  0  0  0  0;
    0  0  0  0  0    0  0  0  0  0;
    0  0  0  0  0    0  0  0  0  0];

%elevation = repmat([0 .1 .2 .1 0 .1 .2 .1 0 0], 10, 1);

route = PathOptimizer(obstacles, 37.25 ,-121.95);