#include "NiftVehicleSetup.h"
#include "PhysicsEngine/PhysicsAsset.h"
#include "PhysicsEngine/BodySetup.h"
#include "PhysicsAssetUtils.h"
#include "Engine/SkeletalMesh.h"
#include "Engine/StaticMesh.h"
#include "Engine/Blueprint.h"
#include "Engine/SCS_Node.h"
#include "Engine/SimpleConstructionScript.h"
#include "Engine/InheritableComponentHandler.h"
#include "Components/StaticMeshComponent.h"
#include "Components/SkinnedMeshComponent.h"
#include "GameFramework/Actor.h"
#include "Engine/CollisionProfile.h"
#include "Engine/Engine.h"
#include "Components/LightComponent.h"
#include "Components/LocalLightComponent.h"
#include "Components/SpotLightComponent.h"
#include "Kismet2/KismetEditorUtilities.h"

DEFINE_LOG_CATEGORY_STATIC(LogNiftVehicleSetup, Log, All);

FTransform UNiftVehicleSetup::GetRefBoneComponentTransform(USkeletalMesh* Mesh, FName Bone)
{
  if (!Mesh) return FTransform::Identity;
  const FReferenceSkeleton& RS = Mesh->RefSkeleton;
  int32 Index = RS.FindBoneIndex(Bone);
  FTransform T = FTransform::Identity;
  while (Index != INDEX_NONE)
  {
    T = T * RS.GetRefBonePose()[Index];
    Index = RS.GetParentIndex(Index);
  }
  return T;
}

static int32 FindOrCreateBody(UPhysicsAsset* PA, USkeletalMesh* Mesh, FName Bone)
{
  int32 Index = PA->FindBodyIndex(Bone);
  if (Index != INDEX_NONE) return Index;
  FPhysAssetCreateParams Params;
  Params.MinBoneSize = 0.f;
  Index = FPhysicsAssetUtils::CreateNewBody(PA, Bone, Params);
  UE_LOG(LogNiftVehicleSetup, Display, TEXT("created body for %s -> %d"), *Bone.ToString(), Index);
  return Index;
}

int32 UNiftVehicleSetup::SetupVehiclePhysicsAsset(UPhysicsAsset* PA, USkeletalMesh* Mesh, UStaticMesh* CollisionMesh,
                                                   FName BodyBone, const TArray<FName>& WheelBones, float WheelRadiusCm)
{
  if (!PA || !Mesh) { UE_LOG(LogNiftVehicleSetup, Error, TEXT("null physics asset or mesh")); return 0; }
  int32 Done = 0;
  PA->Modify();
  // -- body
  {
    const int32 Index = FindOrCreateBody(PA, Mesh, BodyBone);
    if (Index == INDEX_NONE) { UE_LOG(LogNiftVehicleSetup, Error, TEXT("no body for %s"), *BodyBone.ToString()); return 0; }
    USkeletalBodySetup* BS = PA->SkeletalBodySetups[Index];
    BS->Modify();
    BS->RemoveSimpleCollision();
    if (CollisionMesh && CollisionMesh->BodySetup)
    {
      BS->AddCollisionFrom(CollisionMesh->BodySetup);
      const FTransform BoneCS = GetRefBoneComponentTransform(Mesh, BodyBone);
      const FTransform ToBone = BoneCS.Inverse();
      for (FKConvexElem& Elem : BS->AggGeom.ConvexElems) { Elem.SetTransform(Elem.GetTransform() * ToBone); }
      for (FKBoxElem& Elem : BS->AggGeom.BoxElems) { Elem.SetTransform(Elem.GetTransform() * ToBone); }
      for (FKSphereElem& Elem : BS->AggGeom.SphereElems) { Elem.Center = ToBone.TransformPosition(Elem.Center); }
      UE_LOG(LogNiftVehicleSetup, Display, TEXT("body %s: %d convex, %d box, %d sphere from %s (bone cs: %s)"),
             *BodyBone.ToString(), BS->AggGeom.ConvexElems.Num(), BS->AggGeom.BoxElems.Num(), BS->AggGeom.SphereElems.Num(),
             *CollisionMesh->GetName(), *BoneCS.ToString());
    }
    BS->PhysicsType = PhysType_Default;
    BS->CollisionReponse = EBodyCollisionResponse::BodyCollision_Enabled;
    BS->DefaultInstance.bNotifyRigidBodyCollision = true;
    BS->DefaultInstance.LinearDamping = 0.f;
    BS->CollisionTraceFlag = CTF_UseSimpleAsComplex;
    BS->InvalidatePhysicsData(); BS->CreatePhysicsMeshes();
    ++Done;
  }
  // -- wheels
  for (const FName& Bone : WheelBones)
  {
    const int32 Index = FindOrCreateBody(PA, Mesh, Bone);
    if (Index == INDEX_NONE) { UE_LOG(LogNiftVehicleSetup, Error, TEXT("no body for %s"), *Bone.ToString()); continue; }
    USkeletalBodySetup* BS = PA->SkeletalBodySetups[Index];
    BS->Modify();
    BS->RemoveSimpleCollision();
    FKSphereElem Sphere(WheelRadiusCm); Sphere.Center = FVector::ZeroVector;
    BS->AggGeom.SphereElems.Add(Sphere);
    BS->PhysicsType = PhysType_Kinematic;
    BS->CollisionReponse = EBodyCollisionResponse::BodyCollision_Disabled;
    BS->DefaultInstance.bNotifyRigidBodyCollision = true;
    BS->DefaultInstance.LinearDamping = 0.f;
    BS->InvalidatePhysicsData(); BS->CreatePhysicsMeshes();
    ++Done;
  }
  PA->UpdateBodySetupIndexMap();
  PA->UpdateBoundsBodiesArray();
  PA->RefreshPhysicsAssetChange();
  PA->SetPreviewMesh(Mesh);
  PA->MarkPackageDirty();
  UE_LOG(LogNiftVehicleSetup, Display, TEXT("%s: %d bodies configured, %d constraints kept"), *PA->GetName(), Done, PA->ConstraintSetup.Num());
  return Done;
}

bool UNiftVehicleSetup::SetBlueprintComponentStaticMesh(UBlueprint* Blueprint, FName ComponentName, UStaticMesh* Mesh)
{
  if (!Blueprint) return false;
  // walk the blueprint and its blueprint parents
  TArray<UBlueprint*> Chain;
  for (UClass* C = Blueprint->GeneratedClass; C; C = C->GetSuperClass())
  {
    if (UBlueprint* BP = Cast<UBlueprint>(C->ClassGeneratedBy)) Chain.Add(BP);
  }
  for (UBlueprint* BP : Chain)
  {
    if (!BP->SimpleConstructionScript) continue;
    for (USCS_Node* Node : BP->SimpleConstructionScript->GetAllNodes())
    {
      if (!Node || Node->GetVariableName() != ComponentName) continue;
      UStaticMeshComponent* Template = nullptr;
      if (BP == Blueprint)
      {
        Template = Cast<UStaticMeshComponent>(Node->ComponentTemplate);
      }
      else
      {
        UInheritableComponentHandler* ICH = Blueprint->GetInheritableComponentHandler(true);
        if (!ICH) return false;
        const FComponentKey Key(Node);
        UActorComponent* Override = ICH->GetOverridenComponentTemplate(Key);
        if (!Override) Override = ICH->CreateOverridenComponentTemplate(Key);
        Template = Cast<UStaticMeshComponent>(Override);
      }
      if (!Template) { UE_LOG(LogNiftVehicleSetup, Error, TEXT("%s is not a static mesh component"), *ComponentName.ToString()); return false; }
      Template->Modify();
      Template->SetStaticMesh(Mesh);
      Blueprint->MarkPackageDirty();
      UE_LOG(LogNiftVehicleSetup, Display, TEXT("%s.%s (from %s) -> %s"), *Blueprint->GetName(), *ComponentName.ToString(), *BP->GetName(), Mesh ? *Mesh->GetName() : TEXT("null"));
      return true;
    }
  }
  UE_LOG(LogNiftVehicleSetup, Error, TEXT("component %s not found on %s or its parents"), *ComponentName.ToString(), *Blueprint->GetName());
  return false;
}

bool UNiftVehicleSetup::AddBlueprintStaticMeshComponent(UBlueprint* Blueprint, FName ComponentName, UStaticMesh* Mesh, const TArray<FName>& Tags)
{
  if (!Blueprint || !Blueprint->SimpleConstructionScript || !Blueprint->GeneratedClass) return false;
  USimpleConstructionScript* SCS = Blueprint->SimpleConstructionScript;
  USCS_Node* Node = nullptr;
  for (USCS_Node* N : SCS->GetAllNodes()) { if (N && N->GetVariableName() == ComponentName) { Node = N; break; } }
  if (!Node)
  {
    AActor* CDO = Blueprint->GeneratedClass->GetDefaultObject<AActor>();
    USceneComponent* Root = CDO ? CDO->GetRootComponent() : nullptr;
    if (!Root) { UE_LOG(LogNiftVehicleSetup, Error, TEXT("%s has no native root component"), *Blueprint->GetName()); return false; }
    Node = SCS->CreateNode(UStaticMeshComponent::StaticClass(), ComponentName);
    Node->SetParent(Root);
    SCS->AddNode(Node);
    UE_LOG(LogNiftVehicleSetup, Display, TEXT("%s: new component %s under %s"), *Blueprint->GetName(), *ComponentName.ToString(), *Root->GetName());
  }
  UStaticMeshComponent* Template = Cast<UStaticMeshComponent>(Node->ComponentTemplate);
  if (!Template) { UE_LOG(LogNiftVehicleSetup, Error, TEXT("%s is not a static mesh component"), *ComponentName.ToString()); return false; }
  Template->Modify();
  Template->SetStaticMesh(Mesh);
  Template->ComponentTags = Tags;
  Template->SetCollisionEnabled(ECollisionEnabled::NoCollision);
  Template->SetCollisionProfileName(UCollisionProfile::NoCollision_ProfileName);
  Template->SetGenerateOverlapEvents(false);
  Template->CastShadow = false;
  Template->SetRelativeTransform(FTransform::Identity);
  Blueprint->MarkPackageDirty();
  UE_LOG(LogNiftVehicleSetup, Display, TEXT("%s.%s -> %s, %d tags"), *Blueprint->GetName(), *ComponentName.ToString(), Mesh ? *Mesh->GetName() : TEXT("null"), Tags.Num());
  return true;
}

bool UNiftVehicleSetup::CopyBlueprintComponent(UBlueprint* Blueprint, UBlueprint* SourceBlueprint, FName SourceName, FName NewName, FVector RelativeLocation)
{
  if (!Blueprint || !SourceBlueprint || !Blueprint->SimpleConstructionScript || !SourceBlueprint->SimpleConstructionScript) return false;
  USCS_Node* Source = nullptr;
  for (USCS_Node* N : SourceBlueprint->SimpleConstructionScript->GetAllNodes()) { if (N && N->GetVariableName() == SourceName) { Source = N; break; } }
  if (!Source || !Source->ComponentTemplate) { UE_LOG(LogNiftVehicleSetup, Error, TEXT("%s not found on %s"), *SourceName.ToString(), *SourceBlueprint->GetName()); return false; }
  USimpleConstructionScript* SCS = Blueprint->SimpleConstructionScript;
  for (USCS_Node* N : SCS->GetAllNodes()) { if (N && N->GetVariableName() == NewName) { SCS->RemoveNodeAndPromoteChildren(N); break; } }
  AActor* CDO = Blueprint->GeneratedClass ? Blueprint->GeneratedClass->GetDefaultObject<AActor>() : nullptr;
  USceneComponent* Root = CDO ? CDO->GetRootComponent() : nullptr;
  if (!Root) { UE_LOG(LogNiftVehicleSetup, Error, TEXT("%s has no native root component"), *Blueprint->GetName()); return false; }
  USCS_Node* Node = SCS->CreateNode(Source->ComponentTemplate->GetClass(), NewName);
  UEngine::CopyPropertiesForUnrelatedObjects(Source->ComponentTemplate, Node->ComponentTemplate);
  Node->SetParent(Root);
  SCS->AddNode(Node);
  if (USceneComponent* SC = Cast<USceneComponent>(Node->ComponentTemplate)) { SC->SetRelativeLocation(RelativeLocation); }
  Blueprint->MarkPackageDirty();
  UE_LOG(LogNiftVehicleSetup, Display, TEXT("%s.%s <- %s.%s (%s) at %s"), *Blueprint->GetName(), *NewName.ToString(), *SourceBlueprint->GetName(),
         *SourceName.ToString(), *Node->ComponentTemplate->GetClass()->GetName(), *RelativeLocation.ToString());
  return true;
}

FString UNiftVehicleSetup::DescribeBlueprintComponents(UBlueprint* Blueprint)
{
  FString Out;
  if (!Blueprint) return Out;
  for (UClass* C = Blueprint->GeneratedClass; C; C = C->GetSuperClass())
  {
    UBlueprint* BP = Cast<UBlueprint>(C->ClassGeneratedBy);
    if (!BP || !BP->SimpleConstructionScript) continue;
    for (USCS_Node* Node : BP->SimpleConstructionScript->GetAllNodes())
    {
      if (!Node) continue;
      UActorComponent* T = Node->ComponentTemplate;
      FString Line = FString::Printf(TEXT("%s | %s | %s | parent %s%s | socket %s"), *BP->GetName(), *Node->GetVariableName().ToString(),
        T ? *T->GetClass()->GetName() : TEXT("null"), *Node->ParentComponentOrVariableName.ToString(),
        Node->bIsParentComponentNative ? TEXT(" (native)") : TEXT(""), *Node->AttachToName.ToString());
      if (T)
      {
        TArray<FString> TagStrings; for (const FName& Tag : T->ComponentTags) TagStrings.Add(Tag.ToString());
        Line += TEXT(" | tags ") + FString::Join(TagStrings, TEXT(","));
        if (UStaticMeshComponent* S = Cast<UStaticMeshComponent>(T))
        {
          Line += FString::Printf(TEXT(" | mesh %s | collision %d | shadow %d"), S->GetStaticMesh() ? *S->GetStaticMesh()->GetPathName() : TEXT("null"),
                                  (int32)S->GetCollisionEnabled(), S->CastShadow ? 1 : 0);
          TArray<FString> Mats; for (UMaterialInterface* M : S->OverrideMaterials) Mats.Add(M ? M->GetPathName() : TEXT("null"));
          Line += TEXT(" | overrides ") + FString::Join(Mats, TEXT(","));
        }
        if (USceneComponent* SC = Cast<USceneComponent>(T)) Line += TEXT(" | rel ") + SC->GetRelativeTransform().ToString();
        if (ULightComponent* LC = Cast<ULightComponent>(T))
        {
          Line += FString::Printf(TEXT(" | intensity %.1f | color %s | shadows %d | visible %d"), LC->Intensity, *LC->LightColor.ToString(), LC->CastShadows ? 1 : 0, LC->IsVisible() ? 1 : 0);
          if (ULocalLightComponent* LL = Cast<ULocalLightComponent>(LC)) Line += FString::Printf(TEXT(" | radius %.0f"), LL->AttenuationRadius);
          if (USpotLightComponent* SL = Cast<USpotLightComponent>(LC)) Line += FString::Printf(TEXT(" | cone %.0f/%.0f"), SL->InnerConeAngle, SL->OuterConeAngle);
        }
      }
      Out += Line + TEXT("\n");
    }
  }
  return Out;
}

bool UNiftVehicleSetup::CompileBlueprint(UBlueprint* Blueprint)
{
  if (!Blueprint) return false;
  FKismetEditorUtilities::CompileBlueprint(Blueprint);
  const bool bOk = Blueprint->Status != BS_Error;
  UE_LOG(LogNiftVehicleSetup, Display, TEXT("compiled %s: status %d"), *Blueprint->GetName(), (int32)Blueprint->Status);
  return bOk;
}
