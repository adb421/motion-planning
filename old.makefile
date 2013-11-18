all: AStar_test RGRRT_test

AStar_test: AStarNode.cpp AStarTest.cpp
	g++ -std=c++0x -ICGAL AStarNode.cpp AStarTest.cpp -o AStar_test
#	g++ -std=c++0x -msse2 -O2 -fopenmp AStarNode.cpp AStarTest.cpp -o AStar_test

RGRRT_test: RGRRTNode.cpp
	g++ -std=c++0x -msse2 -O2 -fopenmp RGRRTNode.cpp RGRRT_test.cpp -o RGRRT_test

clean:
	 rm -rf *o AStar_test RGRRT_test
