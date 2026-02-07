package edu.ncsu.csc411.ps06.agent;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.HashMap;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.PriorityQueue;

import edu.ncsu.csc411.ps06.environment.Action;
import edu.ncsu.csc411.ps06.environment.Environment;
import edu.ncsu.csc411.ps06.environment.Position;
import edu.ncsu.csc411.ps06.environment.TileStatus;

/**
 * Represents a planning agent within an environment modeled after the Chip's
 * Challenge Windows 95 game. This agent develops a plan for navigating the
 * environment to collect chips and keys in order to reach the environment's
 * portal (goal condition).
 *
 * @author Rohit Kunta
 */
public class Robot {
    private final Environment env;

    /**
     * Constructor for the Robot
     *
     * @param env
     *            the environment it is being loaded into
     */
    public Robot ( final Environment env ) {
        this.env = env;
    }

    /**
     * This is a private Node class I used to represent a node in A* search.
     */
    private class Node {
        Position        pos;
        HashSet<String> inv;   // this is the simulated inventory state
        Node            parent;
        int             g;     // the cost so far of the path
        int             h;     // this is the heuristic cost to the goal
        int             f;     // total cost of the path or g + h

        /**
         * node constructor
         *
         * @param pos
         *            position of the node
         * @param inv
         *            the simulated inventory of the node
         * @param parent
         *            the parent Node of the node
         * @param g
         *            the cost of the path so far
         * @param h
         *            the heuristic cost to the goal
         */
        Node ( final Position pos, final HashSet<String> inv, final Node parent, final int g, final int h ) {
            this.pos = pos;
            this.inv = inv;
            this.parent = parent;
            this.g = g;
            this.h = h;
            this.f = g + h;
        }

        /**
         * creates a unique string signature combining position and sorted
         * inventory.
         *
         * @return the unique string identifying the node.
         */
        String getSignature () {
            final List<String> list = new ArrayList<>( inv );
            Collections.sort( list );
            return pos.getRow() + "," + pos.getCol() + ":" + list.toString();
        }
    }

    /**
     * This is the main method called to determine the next action of the Robot
     * in the map.
     */
    public Action getAction () {
        // gets the current position of the robot
        final Position start = env.getRobotPosition( this );
        // gets the current inventory of the robot
        final HashSet<String> inventory = new HashSet<>( env.getRobotHoldings( this ) );
        // gets the map of all positions in the environment.
        final Map<TileStatus, ArrayList<Position>> envPositions = env.getEnvironmentPositions();
        List<Position> path = null;

        // If there are chips that have not been picked up yet, plan a path to
        // the nearest chip to pick it up
        final ArrayList<Position> chips = envPositions.get( TileStatus.CHIP );
        if ( !chips.isEmpty() ) {
            path = findPathToClosestTarget( start, chips, inventory );
            // If there is not path to a chip, meaning it is likely blocked by a
            // door, then go find the missing key
            if ( path == null ) {
                path = findPathToMissingKey( start, inventory );
            }
        }
        else {
            // the case where all the chips have been collected.
            // THis now plans a path directly to the goal tile, which it didn't
            // before.
            final ArrayList<Position> goals = envPositions.get( TileStatus.GOAL );
            if ( goals != null && !goals.isEmpty() ) {
                final Position goal = goals.get( 0 );
                path = aStarSearch( start, goal, inventory );
                // if there is no clear path to the goal tile, the we look for
                // the missing key because it is likely blocked by a door.
                if ( path == null ) {
                    path = findPathToMissingKey( start, inventory );
                }
            }
        }

        // if a valid path was found and there is atleast one move left, then
        // return the action for that move
        if ( path != null && path.size() > 1 ) {
            final Position nextPos = path.get( 1 );
            return getActionForMove( start, nextPos );
        }
        return Action.DO_NOTHING; // else return nothing
    }

    /**
     * Helper method to convert a move from one position to another into an
     * action, since this is repetitive.
     *
     * @param current
     *            the current position we want to move from
     * @param next
     *            the position we want to move to
     * @return the Action to move from position current to next
     */
    private Action getActionForMove ( final Position current, final Position next ) {
        // the difference in row, determines whether to move up or down
        final int dRow = next.getRow() - current.getRow();
        // the difference in column, determines whether to move left or right
        final int dCol = next.getCol() - current.getCol();

        if ( dRow == -1 ) { // if the next row is up
            return Action.MOVE_UP;
        }
        if ( dRow == 1 ) { // if the next row is down
            return Action.MOVE_DOWN;
        }
        if ( dCol == -1 ) { // if the next col is left
            return Action.MOVE_LEFT;
        }
        if ( dCol == 1 ) { // if the next col is right
            return Action.MOVE_RIGHT;
        }
        return Action.DO_NOTHING;
    }

    /**
     * This finds the shortest path from a given starting position to one of the
     * few target positions using A*.
     *
     * @param start
     *            the initial position
     * @param targets
     *            the list of targets to find paths to
     * @param inventory
     *            the inventory to be simulated in the A* search
     *
     * @return bestPath the path to the closest target.
     */
    private List<Position> findPathToClosestTarget ( final Position start, final List<Position> targets,
            final HashSet<String> inventory ) {
        // keeps track of the shortest path found so far
        List<Position> bestPath = null;
        int bestLength = Integer.MAX_VALUE;
        // iterates through all the targets
        for ( final Position target : targets ) {
            // computes path from start to target
            final List<Position> path = aStarSearch( start, target, inventory );
            if ( path != null && path.size() < bestLength ) {
                bestLength = path.size();
                bestPath = path; // updates bestPath if it is the shortest.
            }
        }
        return bestPath; // returns the closes target
    }

    /**
     * This method finds a path to a missing key the agent does not have yet.
     * This is invoked if the path to a chip or the Goal is blocked by a door
     * that require missing key(s).
     *
     * @param start
     *            the starting position
     *
     * @param inventory
     *            the inventory to be updated in the A* pathfinding
     */
    private List<Position> findPathToMissingKey ( final Position start, final HashSet<String> inventory ) {
        // the types of keys available
        final String[] keyTypes = { "KEY_BLUE", "KEY_GREEN", "KEY_RED", "KEY_YELLOW" };
        List<Position> bestPath = null; // the best (Shortest) path so far
        int bestLength = Integer.MAX_VALUE;

        // the positions and their statuses in the environment
        final Map<TileStatus, ArrayList<Position>> envPositions = env.getEnvironmentPositions();

        // for every key in the list of keys possible, check's the robot's
        // current inventory for that key.
        for ( final String key : keyTypes ) {
            // if the inventory does not contain the key, then it determines the
            // corresponding tileStatus THen it sees in the environment where
            // are all the places that key is located. For each of these
            // positions,it then uses A* to find the path from the start
            // position to the position where that key is located.
            if ( !inventory.contains( key ) ) {
                TileStatus keyStatus = null;
                if ( key.equals( "KEY_BLUE" ) ) {
                    keyStatus = TileStatus.KEY_BLUE;
                }
                else if ( key.equals( "KEY_GREEN" ) ) {
                    keyStatus = TileStatus.KEY_GREEN;
                }
                else if ( key.equals( "KEY_RED" ) ) {
                    keyStatus = TileStatus.KEY_RED;
                }
                else if ( key.equals( "KEY_YELLOW" ) ) {
                    keyStatus = TileStatus.KEY_YELLOW;
                }
                //
                if ( keyStatus != null && envPositions.get( keyStatus ) != null ) {
                    // finding the paths to all the positions where keys are
                    // located, and stores the shortest path
                    for ( final Position keyPos : envPositions.get( keyStatus ) ) {
                        final List<Position> path = aStarSearch( start, keyPos, inventory );
                        if ( path != null && path.size() < bestLength ) {
                            bestLength = path.size();
                            bestPath = path;
                        }
                    }
                }
            }
        }
        return bestPath; // returns the shortest path
    }

    /**
     * the A* search algorithm that plans a path from the start position, to the
     * goal position( not to be confused with the portal), while updating the
     * simulated inventory (which contains the keys we picked up or used) as the
     * path is being planned in A*.
     *
     * @param start
     *            the start position of the path
     * @param the
     *            goal position of the path
     * @param inventory
     *            the inventory to be updated in A*
     *
     * @return the list of positions which represent the path
     */
    private List<Position> aStarSearch ( final Position start, final Position goal, final HashSet<String> inventory ) {

        // the priority queue which is ordered by the nodes total cost f, (g +
        // h)
        final PriorityQueue<Node> open = new PriorityQueue<>( Comparator.comparingInt( n -> n.f ) );
        // keeps track of the visited states with their best- g-cost.
        final HashMap<String, Integer> closed = new HashMap<>();

        // starting node with the robot's current position, inventory, no
        // parent, g cost of 0, using manhattan
        // heuristic from start to goal.
        final Node startNode = new Node( start, new HashSet<>( inventory ), null, 0, manhattan( start, goal ) );
        open.add( startNode ); // adds the startnode to the priority queue.

        // until the queue is empty
        while ( !open.isEmpty() ) {

            // pulls the head of the queue
            final Node current = open.poll();
            // build path if current node is at goal
            if ( current.pos.equals( goal ) ) {
                return reconstructPath( current );
            }

            // the unique string identifier, checks the closed (visited) hashmap
            // to see if an equivalent state is already there with a lower
            // g-cost. If it doesn't, it doesn't process the node.
            final String sig = current.getSignature();
            if ( closed.containsKey( sig ) && closed.get( sig ) <= current.g ) {
                continue;
            }

            // puts the node in the visited map
            closed.put( sig, current.g );

            // creates the neighbors of the current node by simulating moves.
            for ( final Node neighbor : getNeighbors( current, goal ) ) {
                final String neighborSig = neighbor.getSignature();
                // checks the neighbors against the closed set, and if the
                // neighbor's g cost is promising and has a lower g cost,
                if ( closed.containsKey( neighborSig ) && closed.get( neighborSig ) <= neighbor.g ) {
                    continue; // skips processing if it is bad
                }
                open.add( neighbor ); // adds to open set if it is promising.
            }
        }
        return null; // no path found
    }

    /**
     * This is the heuristic for A*, returns the manhattan distance between 2
     * positions
     *
     * @param a
     *            position a
     * @param b
     *            position b
     *
     * @return manhattan distance between a and b
     */
    private int manhattan ( final Position a, final Position b ) {
        return Math.abs( a.getRow() - b.getRow() ) + Math.abs( a.getCol() - b.getCol() );
    }

    /**
     * Rebuilds the path from the given node to the start node
     *
     * @param the
     *            ending node of the path (goal)
     */
    private List<Position> reconstructPath ( Node node ) {
        final LinkedList<Position> path = new LinkedList<>();
        while ( node != null ) {
            path.addFirst( node.pos );
            node = node.parent; // reassigns parent
        }
        return path;
    }

    /**
     * Expands the given node to its neighboring states. This method simulates
     * picking up keys (adding them to inventory) and using keys to pass through
     * doors (removing them).
     *
     * Takes a node and expands the node to its neighboring nodes/states during
     * A*. This simulates picking up keys and using the keys to unlock doors
     *
     * @param current
     *            the current state
     * @param goal
     *            the goal state/node
     *
     * @return the neighboring states/nodes of the current node
     */
    private List<Node> getNeighbors ( final Node current, final Position goal ) {

        // initializes a new arrayList of nodes, then gets all the neighboring
        // positions to the current
        final List<Node> neighbors = new ArrayList<>();
        final Map<String, Position> neighborPositions = env.getNeighborPositions( current.pos );
        // iterates through all the neighboring positions,
        for ( final Position pos : neighborPositions.values() ) {
            if ( !canTraverse( pos, current.inv ) ) { // if you can't traverse
                                                      // it, skip the position
                continue;
            }
            // simulating the new inventory
            final HashSet<String> newInv = new HashSet<>( current.inv );
            // gets status of neighboring tile
            final TileStatus status = env.getTiles().get( pos ).getStatus();
            // if the tile holds a key, pick it up and add to inventory
            if ( status == TileStatus.KEY_BLUE ) {
                newInv.add( "KEY_BLUE" );
            }
            if ( status == TileStatus.KEY_GREEN ) {
                newInv.add( "KEY_GREEN" );
            }
            if ( status == TileStatus.KEY_RED ) {
                newInv.add( "KEY_RED" );
            }
            if ( status == TileStatus.KEY_YELLOW ) {
                newInv.add( "KEY_YELLOW" );
            }
            // if the neighboring tile is a door, simulate removing the
            // corresponding key
            if ( status == TileStatus.DOOR_BLUE ) {
                newInv.remove( "KEY_BLUE" );
            }
            if ( status == TileStatus.DOOR_GREEN ) {
                newInv.remove( "KEY_GREEN" );
            }
            if ( status == TileStatus.DOOR_RED ) {
                newInv.remove( "KEY_RED" );
            }
            if ( status == TileStatus.DOOR_YELLOW ) {
                newInv.remove( "KEY_YELLOW" );
            }

            // finds the new the g-cost of a neighbor
            final int newG = current.g + 1;
            // finds the new heuristic of the neighboring position
            final int newH = manhattan( pos, goal );

            // add the newly constructed neighbor node to the node list
            neighbors.add( new Node( pos, newInv, current, newG, newH ) );
        }

        // return list of node neighbors/states.
        return neighbors;
    }

    /**
     * Determines whether the robot can traverse into the given position with
     * the provided inventory.
     *
     * Checks whether or not the robot can even go to the given position with
     * its current inventory
     *
     * @param pos
     *            the position we want to traverse to
     * @param inv
     *            the inventory we have currently
     *
     * @return true if we can go to pos, false if we can't.
     */
    private boolean canTraverse ( final Position pos, final HashSet<String> inv ) {

        // get status of the position
        final TileStatus status = env.getTiles().get( pos ).getStatus();

        // returns false if the tile is a wall or water.
        if ( status == TileStatus.WALL || status == TileStatus.WATER ) {
            return false;
        }

        // makes sure that there are no more chips remaining if it is Goal Door
        if ( status == TileStatus.DOOR_GOAL && env.getNumRemainingChips() > 0 ) {
            return false;
        }

        // If it is a colored key door, make sure it has the right key
        if ( status == TileStatus.DOOR_BLUE && !inv.contains( "KEY_BLUE" ) ) {
            return false;
        }
        if ( status == TileStatus.DOOR_GREEN && !inv.contains( "KEY_GREEN" ) ) {
            return false;
        }
        if ( status == TileStatus.DOOR_RED && !inv.contains( "KEY_RED" ) ) {
            return false;
        }
        if ( status == TileStatus.DOOR_YELLOW && !inv.contains( "KEY_YELLOW" ) ) {
            return false;
        }

        // else, returns true.
        return true;
    }

    /**
     * Returns a string representing the robot's current position
     */
    @Override
    public String toString () {
        return "Robot [pos=" + env.getRobotPosition( this ) + "]";
    }
}
