package frc.util;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Filesystem;
import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import org.json.simple.JSONArray;
import org.json.simple.JSONObject;
import org.json.simple.parser.JSONParser;

public class LedCustomAnimations {

    private final AddressableLED m_Led;
    private final AddressableLEDBuffer ledBuffer;
    private final String AnimationPath;
    private final boolean isLoop;

    private int Timer;

    JSONArray json;

    public LedCustomAnimations(
            AddressableLED m_Led,
            AddressableLEDBuffer ledBuffer,
            String AnimationPath,
            int startTime,
            boolean isLoop) {
        this.m_Led = m_Led;
        this.ledBuffer = ledBuffer;
        this.AnimationPath = AnimationPath;
        this.isLoop = isLoop;

        this.Timer = -startTime;
        json = loadPath(AnimationPath);
    }

    public int getAnimationLength() {
        return json.toArray().length;
    }

    public void setAnimation() {
        if (Timer < 0) return;
        if (Timer > getAnimationLength() && isLoop) Timer = 0;
        if (Timer > getAnimationLength() && !isLoop) return;

        JSONObject frame = (JSONObject) json.get(Timer);
        Integer red = (Integer) frame.get("red");
        Integer green = (Integer) frame.get("green");
        Integer blue = (Integer) frame.get("blue");

        Double length = (Double) frame.get("length");

        for (int i = 0; i < ledBuffer.getLength() * (length / 100); i++) {
            ledBuffer.setRGB(i, red, green, blue);
        }
        m_Led.setData(ledBuffer);
        m_Led.start();
        Timer++;
    }

    public JSONArray loadPath(String name) {
        try (BufferedReader br =
                new BufferedReader(
                        new FileReader(
                                new File(
                                        Filesystem.getDeployDirectory(), "5829LedAnimations/" + name + ".json")))) {
            StringBuilder fileContentBuilder = new StringBuilder();
            String line;
            while ((line = br.readLine()) != null) {
                fileContentBuilder.append(line);
            }

            String fileContent = fileContentBuilder.toString();
            JSONArray json = (JSONArray) new JSONParser().parse(fileContent);
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
