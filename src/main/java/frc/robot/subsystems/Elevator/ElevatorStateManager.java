package frc.robot.subsystems.Elevator;

import edu.wpi.first.wpilibj2.command.PrintCommand;
import frc.robot.subsystems.Elevator.GraphCommand.GraphCommandNode;


public class ElevatorStateManager {
    Elevator elevator;
    GraphCommand graph = new GraphCommand();

    ElevatorStateManager(Elevator elevator){
        this.elevator = elevator;

        //addallthestates
        
        GraphCommandNode stowed = graph.new GraphCommandNode("Stowed",
        elevator.moveToPose(elevator.kStowed),
        elevator.moveToPose(elevator.kStowed),
        elevator.moveToPose(elevator.kStowed));

        GraphCommandNode stationPickup = graph.new GraphCommandNode("StationPickup",
        elevator.moveToPose(elevator.kStationPickup),
        elevator.moveToPose(elevator.kStationPickup),
        elevator.moveToPose(elevator.kStationPickup));

        GraphCommandNode floorPickup = graph.new GraphCommandNode("FloorPickup",
        elevator.moveToPose(elevator.kFloorPickup),
        elevator.moveToPose(elevator.kFloorPickup),
        elevator.moveToPose(elevator.kFloorPickup));

        GraphCommandNode L1 = graph.new GraphCommandNode("L1", 
        elevator.moveToPose(elevator.kL1), 
        elevator.moveToPose(elevator.kL1), 
        elevator.scoreAtPose(elevator.kL1));

        GraphCommandNode L2 = graph.new GraphCommandNode("L2",
        elevator.moveToPose(elevator.kL2),
        elevator.moveToPose(elevator.kL2), 
        elevator.scoreAtPose(elevator.kL2));

        GraphCommandNode L3 = graph.new GraphCommandNode("L3",
        elevator.moveToPose(elevator.kL3),
        elevator.moveToPose(elevator.kL3), 
        elevator.scoreAtPose(elevator.kL3));

        GraphCommandNode L4 = graph.new GraphCommandNode("L4",
        elevator.moveToPose(elevator.kL4),
        elevator.moveToPose(elevator.kL4), 
        elevator.scoreAtPose(elevator.kL4));

        GraphCommandNode climbing = graph.new GraphCommandNode("Climbing",
        elevator.moveToPose(elevator.kClimbing),
        elevator.moveToPose(elevator.kClimbing),
        elevator.moveToPose(elevator.kClimbing));
    }


    private void addStates(){
        //createallthestates
    
    }
    

    // private void referenceExternalCode(){
    //     graph.setTargetNode(node);
    // }



}