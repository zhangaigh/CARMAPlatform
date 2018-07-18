package gov.dot.fhwa.saxton.carma.message.factory;

import org.jboss.netty.buffer.ChannelBuffer;
import org.ros.internal.message.Message;
import org.ros.message.MessageFactory;
import org.ros.message.Time;

import cav_msgs.ByteArray;
import cav_msgs.GenericLane;
import cav_msgs.IntersectionGeometry;
import cav_msgs.MapData;
import cav_msgs.NodeListXY;
import cav_msgs.NodeOffsetPointXY;
import cav_msgs.NodeXY;
import gov.dot.fhwa.saxton.carma.rosutils.SaxtonLogger;

public class MapMessage implements IMessage<MapData>{
    
    protected static final int MAX_NODE_LIST_SIZE = 63;

    protected SaxtonLogger log;
    protected MessageFactory messageFactory;
    
    public MapMessage(MessageFactory factory, SaxtonLogger logger) {
        this.log            = logger;
        this.messageFactory = factory;
    }

    // Load libasn1c.so external C library
    static {
        try {
            System.loadLibrary("asn1c");
        } catch (Exception e) {
            System.out.println("Exception trapped while trying to load the asn1c library" + e.toString());
            e.printStackTrace();
        }
    }
    
    /**
     * This is the declaration for native method. It will take encoded MAP byte array as input.
     * It will decode the message and set other byte array inputs values. 
     * @return -1 means decode failed; 0 means decode is successful
     */
    public native int decodeMap(byte[] encodedArray, Object map_object, int[] intersectionData, int[] laneIDData,
            int[] ingressApproachData, int[] egressApproachData, int[] laneDirectionData, int[] laneTypeData, int[][] nodeOffsetData);
    
    @Override
    public MessageContainer encode(Message plainMessage) {
        // This class currently does not support encode MAP message 
        throw new UnsupportedOperationException();
    }

    @Override
    public MessageContainer decode(ByteArray binaryMessage) {
        ChannelBuffer buffer = binaryMessage.getContent();
        byte[] encodedMsg = new byte[buffer.capacity()];
        for (int i = 0; i < buffer.capacity(); i++) {
            encodedMsg[i] = buffer.getByte(i);
        }
        int[] intersectionData = new int[9];
        int[] laneIDData = new int[255];
        int[] ingressApproachData = new int[255];
        int[] egressApproachData = new int[255];
        int[] laneDirectionData = new int[255];
        int[] laneTypeData = new int[255];
        int[][] nodeOffsetData = new int[255][189];
        MapData map = messageFactory.newFromType(MapData._TYPE);
        int res = decodeMap(encodedMsg, map, intersectionData, laneIDData, ingressApproachData,
                                    egressApproachData, laneDirectionData, laneTypeData, nodeOffsetData);
        if (res == -1) {
            log.warn("MapMessage cannot be decoded.");
            return new MessageContainer("MAP", null);
        }
        map.setIntersectionsExists(intersectionData[8] == 1);
        if(map.getIntersectionsExists()) {
            IntersectionGeometry intersection = messageFactory.newFromType(IntersectionGeometry._TYPE);
            intersection.getId().setId((short) intersectionData[0]);
            intersection.setRevision((byte) intersectionData[1]);
            intersection.getRefPoint().setLatitude(intersectionData[2]);
            intersection.getRefPoint().setLongitude(intersectionData[3]);
            intersection.getRefPoint().setElevationExists(intersectionData[5] == 1);
            if(intersection.getRefPoint().getElevationExists()) {
                intersection.getRefPoint().setElevation(intersectionData[4]);
            }
            intersection.setLaneWidthExists(intersectionData[7] == 1);
            if(intersection.getLaneWidthExists()) {
                intersection.setLaneWidth((short) intersectionData[6]);
            }
            for(int i = 0; i < laneIDData.length; i++) {
                if(laneIDData[i] == -1) {
                    break;
                }
                GenericLane lane = messageFactory.newFromType(GenericLane._TYPE);
                lane.setLaneId((byte) laneIDData[i]);
                lane.setIngressApproachExists(ingressApproachData[i] != -1);
                if(lane.getIngressApproachExists()) {
                    lane.setIngressApproach((byte) ingressApproachData[i]);
                }
                lane.setEgressApproachExists(egressApproachData[i] != -1);
                if(lane.getEgressApproachExists()) {
                    lane.setEgressApproach((byte) egressApproachData[i]);
                }
                // TODO maybe we need to fix this value
                lane.getLaneAttributes().getDirectionalUse().setLaneDirection((byte) (laneDirectionData[i] >> 6));
                lane.getLaneAttributes().getLaneType().setChoice((byte) (laneTypeData[i] - 1));
                lane.getNodeList().setChoice(NodeListXY.NODE_SET_XY);
                for(int j = 0; j < MAX_NODE_LIST_SIZE - 1; j++) {
                    int nodeType = nodeOffsetData[i][j * 3]; 
                    if(nodeType == 0) {
                        break;
                    }
                    NodeXY node = messageFactory.newFromType(NodeXY._TYPE);
                    switch(nodeType) {
                    case 1:
                        node.getDelta().setChoice(NodeOffsetPointXY.NODE_XY1);
                        node.getDelta().getNodeXy1().setX(nodeOffsetData[i][j * 3 + 1]);
                        node.getDelta().getNodeXy1().setY(nodeOffsetData[i][j * 3 + 2]);
                        break;
                    case 2:
                        node.getDelta().setChoice(NodeOffsetPointXY.NODE_XY2);
                        node.getDelta().getNodeXy2().setX(nodeOffsetData[i][j * 3 + 1]);
                        node.getDelta().getNodeXy2().setY(nodeOffsetData[i][j * 3 + 2]);
                        break;
                    case 3:
                        node.getDelta().setChoice(NodeOffsetPointXY.NODE_XY3);
                        node.getDelta().getNodeXy3().setX(nodeOffsetData[i][j * 3 + 1]);
                        node.getDelta().getNodeXy3().setY(nodeOffsetData[i][j * 3 + 2]);
                        break;
                    case 4:
                        node.getDelta().setChoice(NodeOffsetPointXY.NODE_XY4);
                        node.getDelta().getNodeXy4().setX(nodeOffsetData[i][j * 3 + 1]);
                        node.getDelta().getNodeXy4().setY(nodeOffsetData[i][j * 3 + 2]);
                        break;
                    case 5:
                        node.getDelta().setChoice(NodeOffsetPointXY.NODE_XY5);
                        node.getDelta().getNodeXy5().setX(nodeOffsetData[i][j * 3 + 1]);
                        node.getDelta().getNodeXy5().setY(nodeOffsetData[i][j * 3 + 2]);
                        break;
                    case 6:
                        node.getDelta().setChoice(NodeOffsetPointXY.NODE_XY6);
                        node.getDelta().getNodeXy6().setX(nodeOffsetData[i][j * 3 + 1]);
                        node.getDelta().getNodeXy6().setY(nodeOffsetData[i][j * 3 + 2]);
                        break;
                    case 7:
                        node.getDelta().setChoice(NodeOffsetPointXY.NODE_LATLON);
                        node.getDelta().getNodeLatlon().setLatitude(nodeOffsetData[i][j * 3 + 1]);
                        node.getDelta().getNodeLatlon().setLongitude(nodeOffsetData[i][j * 3 + 2]);
                        break;
                    default:
                        break;
                    }
                    lane.getNodeList().getNodes().getNodeSetXy().add(node);
                }
                intersection.getLaneSet().getLaneList().add(lane);
            }
            map.getIntersections().add(intersection);
            map.getHeader().setFrameId("0");
            map.getHeader().setStamp(Time.fromMillis(System.currentTimeMillis()));
        }
        return new MessageContainer("MAP", map);
    }

}
