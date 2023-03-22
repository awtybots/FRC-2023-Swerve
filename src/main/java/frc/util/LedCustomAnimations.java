package frc.util;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Filesystem;

import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.IOException;
 
import org.json.simple.JSONArray;
import org.json.simple.JSONObject;
import org.json.simple.parser.JSONParser;

public class LedCustomAnimations {

    private final AddressableLED m_Led;
    private final AddressableLEDBuffer ledBuffer;
    private boolean isLoop;

    private int Timer;

    private final JSONArray json;

    public LedCustomAnimations(
            AddressableLED m_Led,
            AddressableLEDBuffer ledBuffer,
            String AnimationPath,
            int startTime,
            boolean isLoop) {
        this.m_Led = m_Led;
        this.ledBuffer = ledBuffer;
        this.isLoop = isLoop;

        this.Timer = -startTime;
        json = loadPath(AnimationPath);
    }

    public int getAnimationLength() {
        return json.toArray().length;
    }

    public void reset(){
        this.Timer = 0;
    }

    public void setLoop(boolean value) {
        this.isLoop = value;
    }

    public void setAnimation() {
        if (Timer < 0) return;
        if (Timer > getAnimationLength() && isLoop) Timer = 0;
        if (Timer > getAnimationLength() && !isLoop) return;

        JSONObject frame = (JSONObject) json.get(Timer);
        int red = (int) frame.get("r");
        int green = (int) frame.get("g");
        int blue = (int) frame.get("b");

        double length = (double) frame.get("length");

        for (int i = 0; i < ledBuffer.getLength() * (length / 100); i++) {
            ledBuffer.setRGB(i, red, green, blue);
        }
        m_Led.setData(ledBuffer);
        m_Led.start();
        Timer++;
    }

    public JSONArray loadPath(String name) {
        JSONParser jsonParser = new JSONParser();
        try (FileReader file = new FileReader(Filesystem.getDeployDirectory() +  "5829LedAnimations/" + name + ".json")) {
            Object obj = jsonParser.parse(file);
            JSONArray json = (JSONArray) obj;
            return json;
        } catch (Exception e) {
            e.printStackTrace();
            return null;
        }
    }

    public boolean isFinished(){
        return Timer > getAnimationLength();
    }
}
