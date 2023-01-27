package org.firstinspires.ftc.teamcode.Pathfinding;

/*
@author Declan J. Scott
Simple BFS pathfinding solution for fields represented as 2d arrays
 */

import java.util.ArrayList;

public class BFS {

    public static Coordinate[] FindPath(char[][] field, Coordinate start, Coordinate end)
    {
        ArrayList<Coordinate> searched = new ArrayList<>(); // Blacklist of coordinates that have already been searched
        ArrayList<Coordinate> searchQueue = new ArrayList<>();
        searchQueue.add(start);

        while (!searchQueue.isEmpty())
        {
            Coordinate activeNode = searchQueue.remove(0);

            // Check if we're at the end
            if (activeNode.Equals(end))
            {
                // Backtrack through parents to return path
                ArrayList<Coordinate> reversedPath = new ArrayList<>();
                reversedPath.add(activeNode);
                while (activeNode.parent != null)
                {
                    reversedPath.add(activeNode.parent);
                    activeNode = activeNode.parent;
                }

                // Reverse the path to normal order
                ArrayList<Coordinate> normalPath = new ArrayList<>();
                for (int i = reversedPath.size() - 1; i >= 0; i--) {

                    // Append the elements in reverse order
                    normalPath.add(reversedPath.get(i));
                }

                // Return the path
                Coordinate[] output = new Coordinate[normalPath.size()];
                normalPath.toArray(output);
                return output;
            }else{
                // Add neighbors to queue
                Coordinate[] activeNeighbors = activeNode.getNeighbors(field);
                for (Coordinate neighbor : activeNeighbors) {

                    // Check if the neighbor has been searched or will be searched
                    boolean beenSearched = false;
                    for (Coordinate searchedNode : searched) {
                        if (neighbor.Equals(searchedNode)) {
                            beenSearched = true;
                            break;
                        }
                    }
                    for (Coordinate futureNode : searchQueue) {
                        if (neighbor.Equals(futureNode)) {
                            beenSearched = true;
                            break;
                        }
                    }

                    // Add to the queue
                    if(!beenSearched)
                    {
                        neighbor.parent = activeNode; // Setup parent
                        searchQueue.add(neighbor);
                    }

                    // Add active node to searched list
                    searched.add(activeNode);
                }
            }
        }
        // All nodes have been searched without finding the end. No path exists
        return null;
    }

    // Test function (will crash if run in Android Studio)
    public static void main(String[] args) {
        char[][] field = {
                {'_', '_', '_', '_'},
                {'_', '_', '_', '_'},
                {'_', '_', '_', '_'},
                {'_', '_', '_', '_'},
        };

        Coordinate test = new Coordinate(1, 1);
        Coordinate topLeft = new Coordinate(0, 0);
        Coordinate topCenter = new Coordinate(0, 2);
        Coordinate bottomRight = new Coordinate(3, 3);

        System.out.println(test.getNeighbors(field).length);
        System.out.println(topLeft.getNeighbors(field).length);
        System.out.println(topCenter.getNeighbors(field).length);
        System.out.println(bottomRight.getNeighbors(field).length);

        // Testing BFS
        System.out.println("\nPATHFINDING FROM (0,0) to (3,3): ");
        Coordinate[] path = BFS.FindPath(field, topLeft, bottomRight);
        assert path != null;
        for (Coordinate coordinate : path) {
            System.out.println(coordinate.toString());
        }
    }
}
