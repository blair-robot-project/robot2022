package frc.team449.jacksonWrappers;

import com.fasterxml.jackson.annotation.JsonCreator;
import com.fasterxml.jackson.annotation.JsonProperty;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import java.util.List;

public class MappedTranslationSet {
  final Pose2d startingPose;

  final Pose2d endingPose;

  final List<Translation2d> translations;

  /**
   * Pose2d wrapper for Trajectory loading from map
   *
   * @param startingPose The absolute x position in meters
   * @param translations The angle at this position in degrees
   * @param endingPose The absolute y position in meters
   */
  @JsonCreator
  public MappedTranslationSet(
      @JsonProperty(required = true) final Pose2d startingPose,
      @JsonProperty final List<Translation2d> translations,
      @JsonProperty(required = true) final Pose2d endingPose) {
    this.startingPose = startingPose;
    this.endingPose = endingPose;
    this.translations = translations;
  }

  public Pose2d getStartingPose() {
    return this.startingPose;
  }

  public Pose2d getEndingPose() {
    return this.endingPose;
  }

  public List<Translation2d> getTranslations() {
    return this.translations;
  }
}
