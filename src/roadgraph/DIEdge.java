package roadgraph;

import geography.GeographicPoint;

/**
 * Created by sherxon on 3/24/17.
 */
public class DIEdge {
    private GeographicPoint from,  to;
    private String roadName,  roadType;
    private double length;

    public DIEdge(GeographicPoint from, GeographicPoint to, String roadName, String roadType, double length) {
        this.from = from;
        this.to = to;
        this.roadName = roadName;
        this.roadType = roadType;
        this.length = length;
    }

    public double getLength() {
        return length;
    }

    public void setLength(double length) {
        this.length = length;
    }

    public GeographicPoint getFrom() {
        return from;
    }

    public void setFrom(GeographicPoint from) {
        this.from = from;
    }

    public GeographicPoint getTo() {
        return to;
    }

    public void setTo(GeographicPoint to) {
        this.to = to;
    }

    @Override
    public boolean equals(Object o) {
        if (this == o) return true;
        if (o == null || getClass() != o.getClass()) return false;

        DIEdge edge = (DIEdge) o;

        if (Double.compare(edge.length, length) != 0) return false;
        if (from != null ? !from.equals(edge.from) : edge.from != null) return false;
        if (to != null ? !to.equals(edge.to) : edge.to != null) return false;
        if (roadName != null ? !roadName.equals(edge.roadName) : edge.roadName != null) return false;
        return !(roadType != null ? !roadType.equals(edge.roadType) : edge.roadType != null);

    }

    @Override
    public int hashCode() {
        int result;
        long temp;
        result = from != null ? from.hashCode() : 0;
        result = 31 * result + (to != null ? to.hashCode() : 0);
        result = 31 * result + (roadName != null ? roadName.hashCode() : 0);
        result = 31 * result + (roadType != null ? roadType.hashCode() : 0);
        temp = Double.doubleToLongBits(length);
        result = 31 * result + (int) (temp ^ (temp >>> 32));
        return result;
    }
}
