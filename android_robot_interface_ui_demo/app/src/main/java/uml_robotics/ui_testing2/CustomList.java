package uml_robotics.ui_testing2;


import android.app.Activity;
import android.view.LayoutInflater;
import android.view.View;
import android.view.ViewGroup;
import android.widget.ArrayAdapter;
import android.widget.ImageView;
import android.widget.TextView;

public class CustomList extends ArrayAdapter<String> {
    private final Activity Context;
    private final String[] ListItemsName;
    private final Integer[] ImageName;

    public CustomList(Activity context, String[] content, Integer[] ImageName) {
        super(context, R.layout.list_resource, content);

        this.Context = context;
        this.ListItemsName = content;
        this.ImageName = ImageName;
    }

    @Override
    public View getView(int position, View view, ViewGroup parent) {
        LayoutInflater inflater = Context.getLayoutInflater();
        View ListViewSingle = inflater.inflate(R.layout.list_resource, null, true);

        TextView ListViewItems = (TextView) ListViewSingle.findViewById(R.id.textView);
        ImageView ListViewImage = (ImageView) ListViewSingle.findViewById(R.id.imageView);

        ListViewItems.setText(ListItemsName[position]);
        ListViewImage.setImageResource(ImageName[position]);
        return ListViewSingle;
    }
}

