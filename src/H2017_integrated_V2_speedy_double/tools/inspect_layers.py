"""H2017 USD 레이어 스택 점검. Isaac Sim의 python.sh로 실행한다."""
import sys
from pxr import Sdf, Usd

root = sys.argv[1]
stage = Usd.Stage.Open(root, load=Usd.Stage.LoadNone)

print("=" * 70)
print("1) 루트 레이어 스택 (sublayer 체인)")
print("=" * 70)
for layer in stage.GetLayerStack():
    print(f"  {layer.identifier}")

print()
print("=" * 70)
print("2) reference / payload 를 가진 prim 전체")
print("=" * 70)
for prim in stage.TraverseAll():
    spec = prim.GetPrimStack()
    refs, pays = [], []
    for s in spec:
        for item in s.referenceList.GetAddedOrExplicitItems():
            refs.append(item.assetPath or "<internal>")
        for item in s.payloadList.GetAddedOrExplicitItems():
            pays.append(item.assetPath or "<internal>")
    if refs or pays:
        print(f"\n  {prim.GetPath()}")
        for r in refs:
            print(f"      ref     : {r}")
        for p in pays:
            print(f"      payload : {p}")

print()
print("=" * 70)
print("3) 외부 URL(http/omniverse) 참조 — 오프라인 실행 위험")
print("=" * 70)
found = False
for prim in stage.TraverseAll():
    for s in prim.GetPrimStack():
        items = list(s.referenceList.GetAddedOrExplicitItems())
        items += list(s.payloadList.GetAddedOrExplicitItems())
        for item in items:
            path = item.assetPath or ""
            if path.startswith(("http://", "https://", "omniverse://")):
                print(f"  {prim.GetPath()}  ->  {path}")
                found = True
if not found:
    print("  없음")

print()
print("=" * 70)
print("4) 미해결(파일 없음) 참조")
print("=" * 70)
missing = False
for prim in stage.TraverseAll():
    for s in prim.GetPrimStack():
        items = list(s.referenceList.GetAddedOrExplicitItems())
        items += list(s.payloadList.GetAddedOrExplicitItems())
        for item in items:
            path = item.assetPath or ""
            if not path or path.startswith(("http", "omniverse")):
                continue
            resolved = s.layer.ComputeAbsolutePath(path)
            if not Sdf.Layer.FindOrOpen(resolved):
                print(f"  {prim.GetPath()}  ->  {path}  (해석: {resolved})")
                missing = True
if not missing:
    print("  없음")

print()
print("=" * 70)
print("5) 주요 prim 경로 존재 확인")
print("=" * 70)
for path in [
    "/World",
    "/World/pallet",
    "/World/Cube",
    "/World/h2017_gripper_v2",
    "/World/h2017_gripper_v2_01",
    "/World/ConveyorTrack",
]:
    prim = stage.GetPrimAtPath(path)
    print(f"  {'OK  ' if prim.IsValid() else 'MISS'}  {path}")
