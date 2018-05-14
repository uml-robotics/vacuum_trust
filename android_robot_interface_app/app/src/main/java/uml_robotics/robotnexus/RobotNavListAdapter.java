package uml_robotics.robotnexus;

import android.app.Activity;
import android.view.LayoutInflater;
import android.view.View;
import android.view.ViewGroup;
import android.widget.ArrayAdapter;
import android.widget.ImageView;
import android.widget.TextView;

/**
 * custom adapter for robot selector and navigation drawer lists
 */

public class RobotNavListAdapter extends ArrayAdapter<Robot> {
    private final Activity Context;
    private final Robot[] robots;
    private final Integer[] images;
    private final Integer[] icons;

    public RobotNavListAdapter(Activity context, Robot[] content, Integer[] images, Integer[] icons) {
        super(context, R.layout.robot_list_resource, content);

        this.Context = context;
        this.robots = content;
        this.images = images;
        this.icons = icons;
    }

    @Override
    public View getView(int position, View view, ViewGroup parent) {
        LayoutInflater inflater = Context.getLayoutInflater();
        View ListViewSingle = inflater.inflate(R.layout.robot_list_resource, null, true);

        TextView ListViewItem = (TextView) ListViewSingle.findViewById(R.id.robot_list_text);
        ImageView ListViewImage = (ImageView) ListViewSingle.findViewById(R.id.robot_list_image);
        ImageView ListViewIcon = (ImageView) ListViewSingle.findViewById(R.id.robot_list_icon);

        ListViewItem.setText(robots[position].toString());
        //safety-net for when our first update is an "ack"
        //if (images[position] != null) {
        ListViewImage.setImageResource(images[position]);
        //}
        ListViewIcon.setImageResource(icons[position]);
        return ListViewSingle;
    }
}