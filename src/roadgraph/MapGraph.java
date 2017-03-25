/**
 * @author UCSD MOOC development team and YOU
 * 
 * A class which reprsents a graph of geographic locations
 * Nodes in the graph are intersections between 
 *
 */
package roadgraph;


import geography.GeographicPoint;
import util.GraphLoader;

import java.util.*;
import java.util.function.Consumer;

/**
 * @author UCSD MOOC development team and YOU
 * 
 * A class which represents a graph of geographic locations
 * Nodes in the graph are intersections between 
 *
 */
public class MapGraph {
    Map<GeographicPoint, Set<DIEdge>> map= new HashMap<>();
    int edgeCount=0;
	/** 
	 * Create a new empty MapGraph 
	 */
	public MapGraph() {
	}
	
	/**
	 * Get the number of vertices (road intersections) in the graph
	 * @return The number of vertices in the graph.
	 */
	public int getNumVertices()
	{
		return  map.keySet().size();
	}
	
	/**
	 * Return the intersections, which are the vertices in this graph.
	 * @return The vertices in this graph as GeographicPoints
	 */
	public Set<GeographicPoint> getVertices()
	{
		return new HashSet<>(map.keySet());
	}
	
	/**
	 * Get the number of road segments in the graph
	 * @return The number of edges in the graph.
	 */
	public int getNumEdges()
	{
		return edgeCount;
	}

	
	
	/** Add a node corresponding to an intersection at a Geographic Point
	 * If the location is already in the graph or null, this method does 
	 * not change the graph.
	 * @param location  The location of the intersection
	 * @return true if a node was added, false if it was not (the node
	 * was already in the graph, or the parameter is null).
	 */
	public boolean addVertex(GeographicPoint location) {
		if(location==null || map.containsKey(location))return false;
		 map.put(location, new HashSet<>());
        return true;
	}
	
	/**
	 * Adds a directed edge to the graph from pt1 to pt2.  
	 * Precondition: Both GeographicPoints have already been added to the graph
	 * @param from The starting point of the edge
	 * @param to The ending point of the edge
	 * @param roadName The name of the road
	 * @param roadType The type of the road
	 * @param length The length of the road, in km
	 * @throws IllegalArgumentException If the points have not already been
	 *   added as nodes to the graph, if any of the arguments is null,
	 *   or if the length is less than 0.
	 */
	public void addEdge(GeographicPoint from, GeographicPoint to, String roadName, String roadType, double length)
            throws IllegalArgumentException {
        if(length<0 || from==null || to==null || roadName==null || roadType==null ||
                !map.containsKey(from) ||
                !map.containsKey(to)
                )throw new IllegalArgumentException();

        DIEdge edge= new DIEdge(from, to, roadName, roadType, length);
        map.get(from).add(edge);
        edgeCount++;
	}
	

	/** Find the path from start to goal using breadth first search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @return The list of intersections that form the shortest (unweighted)
	 *   path from start to goal (including both start and goal).
	 */
	public List<GeographicPoint> bfs(GeographicPoint start, GeographicPoint goal) {
		// Dummy variable for calling the search algorithms
        Consumer<GeographicPoint> temp = (x) -> {};
        return bfs(start, goal, temp);
	}
	
	/** Find the path from start to goal using breadth first search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @param nodeSearched A hook for visualization.  See assignment instructions for how to use it.
	 * @return The list of intersections that form the shortest (unweighted)
	 *   path from start to goal (including both start and goal).
	 */
	public List<GeographicPoint> bfs(GeographicPoint start, 
			 					     GeographicPoint goal, Consumer<GeographicPoint> nodeSearched)
	{


        Map<GeographicPoint, GeographicPoint> parent= new HashMap<>();
        parent.put(start, null);
        Queue<GeographicPoint> queue= new LinkedList<>();
        Set<GeographicPoint> visited= new HashSet<>();
        visited.add(start);
        queue.add(start);
        while (!queue.isEmpty()){
            GeographicPoint current= queue.remove();
            nodeSearched.accept(current);
            for (DIEdge edge : map.get(current)) {
                if(!visited.contains(edge.getTo())){
                    queue.add(edge.getTo());
                    visited.add(edge.getTo());
                    parent.put(edge.getTo(), current);
                }
            }
        }
        LinkedList<GeographicPoint> list=new LinkedList<>();
        GeographicPoint point=goal;
        while (point!=null){
            list.addFirst(point);
            point=parent.get(point);
        }

		// Hook for visualization.  See writeup.
		//nodeSearched.accept(next.getLocation());

		return list.size()<=1? null : list;
	}
	

	/** Find the path from start to goal using Dijkstra's algorithm
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> dijkstra(GeographicPoint start, GeographicPoint goal) {
		// Dummy variable for calling the search algorithms
		// You do not need to change this method.
        Consumer<GeographicPoint> temp = (x) -> {};
        return dijkstra(start, goal, temp);
	}
	
	/** Find the path from start to goal using Dijkstra's algorithm
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @param nodeSearched A hook for visualization.  See assignment instructions for how to use it.
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> dijkstra(GeographicPoint start, 
										  GeographicPoint goal, Consumer<GeographicPoint> nodeSearched) {

        Map<GeographicPoint, GeographicPoint> parent= new HashMap<>();
        Map<GeographicPoint, Double> distance= new HashMap<>();
        Set<GeographicPoint> q= new HashSet<>();
        Set<GeographicPoint> visited= new HashSet<>();
        distance.put(start, 0.0);
        q.add(start);

        while (!q.isEmpty()){
            GeographicPoint current=null;
            double min=Double.MAX_VALUE;
            for (GeographicPoint point : q) {
                if(distance.get(point)<=min){
                    current=point;
                    min=distance.get(point);
                }
            }
            q.remove(current);
            nodeSearched.accept(current);
            if(current.equals(goal))return makePath(start, goal, parent);
            if(!visited.contains(current))
                visited.add(current);

            for (DIEdge edge : map.get(current)) {
                if(!visited.contains(edge.getTo()) &&
                        distance.get(current) + edge.getLength() <
                                distance.getOrDefault(edge.getTo(), Double.MAX_VALUE)){
                    distance.put(edge.getTo(), distance.get(current) + edge.getLength());
                    parent.put(edge.getTo(), current);
                    q.add(edge.getTo());
                }
            }
        }
        List<GeographicPoint> list=makePath(start, goal, parent);
        return list.size()<=1? null : list;

	}

	/** Find the path from start to goal using A-Star search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> aStarSearch(GeographicPoint start, GeographicPoint goal) {
		// Dummy variable for calling the search algorithms
        Consumer<GeographicPoint> temp = (x) -> {};
        return aStarSearch(start, goal, temp);
	}
	
	/** Find the path from start to goal using A-Star search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @param nodeSearched A hook for visualization.  See assignment instructions for how to use it.
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> aStarSearch(GeographicPoint start, 
											 GeographicPoint goal, Consumer<GeographicPoint> nodeSearched)
	{
        Map<GeographicPoint, GeographicPoint> parent= new HashMap<>();
        Map<GeographicPoint, Double> distance= new HashMap<>();
        Set<GeographicPoint> openSet= new HashSet<>();
        Set<GeographicPoint> closedSet= new HashSet<>();
        distance.put(start, 0.0);
        openSet.add(start);

        while (!openSet.isEmpty()){
            GeographicPoint current=null;
            double min=Double.MAX_VALUE;
            for (GeographicPoint point : openSet) {
                if(distance.get(point)<min){
                    current=point;
                    min=distance.get(point);
                }
            }
            nodeSearched.accept(current);
            if(current.equals(goal))return makePath(start, goal, parent);
            openSet.remove(current);
            closedSet.add(current);

            for (DIEdge edge : map.get(current)) {
                if(closedSet.contains(edge.getTo()))continue;

                double nextDistance=(distance.get(current) + edge.getLength());
                double heauristicDis=nextDistance + goal.distance(edge.getTo());

                if(heauristicDis < distance.getOrDefault(edge.getTo(), Double.MAX_VALUE)){
                    distance.put(edge.getTo(), heauristicDis);
                    parent.put(edge.getTo(), current);
                    openSet.add(edge.getTo());
                }
            }
        }
        List<GeographicPoint> list=makePath(start, goal, parent);
        return list.size()<=1 ? null : list;
	}

    private List<GeographicPoint> makePath(GeographicPoint start, GeographicPoint goal,
                                           Map<GeographicPoint, GeographicPoint> parent) {

        LinkedList<GeographicPoint> list=new LinkedList<>();
        GeographicPoint point=goal;
        while (point!=null){
            list.addFirst(point);
            point=parent.get(point);
        }
        return list;
    }


    public static void main(String[] args)
	{
		System.out.print("Making a new map...");
		MapGraph firstMap = new MapGraph();
		System.out.print("DONE. \nLoading the map...");
		GraphLoader.loadRoadMap("data/testdata/simpletest.map", firstMap);
		System.out.println("DONE.");
//
        // You can use this method for testing.
		
		
		/* Here are some test cases you should try before you attempt
		 * the Week 3 End of Week Quiz, EVEN IF you score 100% on the 
		 * programming assignment.
		*//**/
		MapGraph simpleTestMap = new MapGraph();
		GraphLoader.loadRoadMap("data/testdata/simpletest.map", simpleTestMap);
		
		GeographicPoint testStart = new GeographicPoint(1.0, 1.0);
		GeographicPoint testEnd = new GeographicPoint(8.0, -1.0);
		
		System.out.println("Test 1 using simpletest: Dijkstra should be 9 and AStar should be 5");
		List<GeographicPoint> testroute = simpleTestMap.dijkstra(testStart,testEnd);
		List<GeographicPoint> testroute2 = simpleTestMap.aStarSearch(testStart,testEnd);
		
		
		MapGraph testMap = new MapGraph();
		GraphLoader.loadRoadMap("data/maps/utc.map", testMap);
		
		// A very simple test using real data
		testStart = new GeographicPoint(32.869423, -117.220917);
		testEnd = new GeographicPoint(32.869255, -117.216927);
		System.out.println("Test 2 using utc: Dijkstra should be 13 and AStar should be 5");
		testroute = testMap.dijkstra(testStart,testEnd);
		testroute2 = testMap.aStarSearch(testStart,testEnd);
		
		
		// A slightly more complex test using real data
		testStart = new GeographicPoint(32.8674388, -117.2190213);
		testEnd = new GeographicPoint(32.8697828, -117.2244506);
		System.out.println("Test 3 using utc: Dijkstra should be 37 and AStar should be 10");
		testroute = testMap.dijkstra(testStart,testEnd);
		testroute2 = testMap.aStarSearch(testStart,testEnd);

		
		
		/* Use this code in Week 3 End of Week Quiz */
		/*MapGraph theMap = new MapGraph();
		System.out.print("DONE. \nLoading the map...");
		GraphLoader.loadRoadMap("data/maps/utc.map", theMap);
		System.out.println("DONE.");

		GeographicPoint start = new GeographicPoint(32.8648772, -117.2254046);
		GeographicPoint end = new GeographicPoint(32.8660691, -117.217393);
		
		
		List<GeographicPoint> route = theMap.dijkstra(start,end);
		List<GeographicPoint> route2 = theMap.aStarSearch(start,end);

		*/
		
	}
	
}
