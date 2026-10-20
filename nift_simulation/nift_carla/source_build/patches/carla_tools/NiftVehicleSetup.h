// NiFT (2026-10): editor helpers that Unreal 4.26's Python cannot reach: reshape a vehicle physics asset
// (convex body from a collision static mesh, kinematic spheres for the wheels), swap a static mesh on a
// blueprint-added component (the CARLA "CustomCollision" raycast mesh), add a tagged static mesh component (the
// "emissive" light mesh CARLA's vehicle blueprint drives), list a blueprint's components and compile a blueprint.
#pragma once
#include "CoreMinimal.h"
#include "Kismet/BlueprintFunctionLibrary.h"
#include "NiftVehicleSetup.generated.h"

class UPhysicsAsset; class USkeletalMesh; class UStaticMesh; class UBlueprint;

UCLASS()
class CARLATOOLS_API UNiftVehicleSetup : public UBlueprintFunctionLibrary
{
  GENERATED_BODY()
public:
  /** Body for BodyBone = convex pieces copied from CollisionMesh (component space -> bone space), default physics,
   *  hit events on, damping 0. Every bone in WheelBones = one sphere of WheelRadiusCm, kinematic, collision disabled.
   *  Missing bodies are created. Returns the number of bodies configured. */
  UFUNCTION(BlueprintCallable, Category = "NiFT")
  static int32 SetupVehiclePhysicsAsset(UPhysicsAsset* PhysicsAsset, USkeletalMesh* Mesh, UStaticMesh* CollisionMesh,
                                        FName BodyBone, const TArray<FName>& WheelBones, float WheelRadiusCm);

  /** Set the StaticMesh of a UStaticMeshComponent that a blueprint (or one of its blueprint parents) adds in its
   *  construction script. Inherited components get an override in the child blueprint. */
  UFUNCTION(BlueprintCallable, Category = "NiFT")
  static bool SetBlueprintComponentStaticMesh(UBlueprint* Blueprint, FName ComponentName, UStaticMesh* Mesh);

  /** Add (or update) a UStaticMeshComponent named ComponentName in the blueprint's own construction script,
   *  attached to the native root component (a vehicle's VehicleMesh), with Mesh, the given component tags, no
   *  collision and no shadow. CARLA's BaseVehiclePawn sets the light parameters on components tagged "emissive". */
  UFUNCTION(BlueprintCallable, Category = "NiFT")
  static bool AddBlueprintStaticMeshComponent(UBlueprint* Blueprint, FName ComponentName, UStaticMesh* Mesh, const TArray<FName>& Tags);

  /** Copy construction-script component SourceName of SourceBlueprint (e.g. a stock vehicle's light component, with all its
   *  settings) into Blueprint's own construction script as NewName, attached to the native root component, at
   *  RelativeLocation (rotation and scale stay the source's). An existing NewName is replaced. CARLA's BaseVehiclePawn groups light components by their name
   *  (front|back-position|low_beam|high_beam|brake|blinker|reverse|fog-l|r-N) and drives them with the light state. */
  UFUNCTION(BlueprintCallable, Category = "NiFT")
  static bool CopyBlueprintComponent(UBlueprint* Blueprint, UBlueprint* SourceBlueprint, FName SourceName, FName NewName, FVector RelativeLocation);

  /** One line per construction-script component of the blueprint and its blueprint parents: owner, name, class,
   *  parent, socket, tags, mesh, materials (for checks from Python). */
  UFUNCTION(BlueprintCallable, Category = "NiFT")
  static FString DescribeBlueprintComponents(UBlueprint* Blueprint);

  UFUNCTION(BlueprintCallable, Category = "NiFT")
  static bool CompileBlueprint(UBlueprint* Blueprint);

  /** Component-space transform of a bone in the mesh reference pose (for checks from Python). */
  UFUNCTION(BlueprintCallable, Category = "NiFT")
  static FTransform GetRefBoneComponentTransform(USkeletalMesh* Mesh, FName Bone);
};
