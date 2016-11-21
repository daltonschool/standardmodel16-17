package org.firstinspires.ftc.teamcode.options;

import android.util.Log;

import java.io.IOException;
import java.lang.reflect.Field;
import java.util.Map;

import fi.iki.elonen.NanoHTTPD;

public class OptionsHTTPD extends NanoHTTPD {
    public OptionsHTTPD() throws IOException, IOException {
        super(9372);
        start(NanoHTTPD.SOCKET_READ_TIMEOUT, false);
    }

    public String createOptionsHtml() {
        String response = "<html>" +
                "<body>" +
                "<h1>Options</h1>" +
                "<form action='/set' method='GET'>";

        Options currentOptions = OptionManager.load();
        Class c = Options.class;
        Field[] fields = c.getDeclaredFields();
        for (Field f : fields) {
            if (f.getName().equals("$change")) {
                // idk what this is, ignore it
                continue;
            }
            if (f.getType() == boolean.class) {
                response += "<input type='checkbox' name='";
                response += f.getName(); // this code makes me feel icky and if your field contained HTML you'd have an XSS but that's ok right
                response += "'";
                try {
                    if ((boolean)f.get(currentOptions)) {
                        response += " checked";
                    }
                } catch (IllegalAccessException e) {
                    e.printStackTrace();
                }
                response += ">";
            } else {
                response += "unknown class! ";
            }
            response += "<label for='";
            response += f.getName(); // see above comment
            response += "'>";
            response += OptionManager.getPrettyName(f.getName()); // see above comment
            response += "</label><br />";
        }

        response += "<input type='submit'>" +
                "</form>" +
                "<style>body{font-family:sans-serif;} label{font-size: 20px;}</style></body>" +
                "</html>";

        return response;
    }

    @Override
    public Response serve(IHTTPSession session) {
        if (session.getUri().equals("/") || session.getUri().equals("/index")) {
            return newFixedLengthResponse(createOptionsHtml());
        } else if (session.getUri().equals("/set")) {
            Map<String, String> parms = session.getParms();
            Options newOptions = new Options();
            Class c = Options.class;
            Field[] fields = c.getDeclaredFields();
            Log.i("options", parms.toString());
            for (Field f : fields) {
                if (f.getName().equals("$change")) {
                    // idk what this is, ignore it
                    continue;
                }
                if (!parms.containsKey(f.getName()) && f.getType() != boolean.class) {
                    return newFixedLengthResponse("Missing param: " + f.getName() + "<br><a href=/>Back</a>");
                }
                Object newVal = null;
                if (f.getType() == boolean.class) {
                    if (parms.containsKey(f.getName()) && parms.get(f.getName()).equals("on")) {
                        newVal = true;
                    } else {
                        newVal = false;
                    }
                }
                if (newVal == null) {
                    return newFixedLengthResponse("Param null: " + f.getName() + "<br><a href=/>Back</a>");
                }
                try {
                    f.set(newOptions, newVal);
                } catch (IllegalAccessException e) {
                    return newFixedLengthResponse("IAE: " + f.getName() + "<br><a href=/>Back</a>");
                }
            }
            OptionManager.save(newOptions);
            return newFixedLengthResponse("Options have been set.<br><a href=/>Back</a>");
        }

        return newFixedLengthResponse(Response.Status.NOT_FOUND, "text/html", "I'm not sure, what do you think?");
    }
}
