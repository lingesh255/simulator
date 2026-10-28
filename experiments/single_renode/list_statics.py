# Renode monitor script (IronPython): `include @list_statics.py` writes every static
# field of the loaded Renode assemblies to $RENODE_STATICS_OUT, one per line:
#   assembly | type | kind | field type | name
# kind is "mutable" or "readonly" (a readonly collection is still shared state).
# Task 17: diff the standalone's build against an official one.
import System
from System.Reflection import BindingFlags
path = System.Environment.GetEnvironmentVariable("RENODE_STATICS_OUT")
flags = BindingFlags.Static | BindingFlags.Public | BindingFlags.NonPublic | BindingFlags.DeclaredOnly
lines = []
for asm in System.AppDomain.CurrentDomain.GetAssemblies():
    name = asm.GetName().Name
    if name.startswith("System") or name.startswith("Microsoft") or name in ("mscorlib", "netstandard"):
        continue
    try:
        types = asm.GetTypes()
    except Exception:
        continue
    for t in types:
        full = t.FullName or ""
        if "<>" in full or "<PrivateImplementationDetails>" in full:
            continue
        for f in t.GetFields(flags):
            if f.IsLiteral or "<>" in f.Name:
                continue
            lines.append("%s | %s | %s | %s | %s" % (name, full, "readonly" if f.IsInitOnly else "mutable", f.FieldType.Name, f.Name))
lines.sort()
System.IO.File.WriteAllLines(path, System.Array[str](lines))
print("wrote %d static fields to %s" % (len(lines), path))
