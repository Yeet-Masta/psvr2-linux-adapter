import ghidra.app.script.GhidraScript;
import ghidra.app.decompiler.*;
import ghidra.program.model.listing.*;
import ghidra.program.model.data.*;
import ghidra.program.model.symbol.*;
import java.io.*;
public class DumpAll extends GhidraScript {
  public void run() throws Exception {
    String out = getScriptArgs()[0];
    DecompInterface d = new DecompInterface();
    d.openProgram(currentProgram);
    try (PrintWriter w = new PrintWriter(new FileWriter(out + "/decomp.c"))) {
      for (Function f : currentProgram.getFunctionManager().getFunctions(true)) {
        if (monitor.isCancelled()) break;
        DecompileResults r = d.decompileFunction(f, 60, monitor);
        w.println("// ==== " + f.getName() + " @ " + f.getEntryPoint());
        if (r.decompileCompleted()) w.println(r.getDecompiledFunction().getC());
        else w.println("// decompile failed: " + r.getErrorMessage());
      }
    }
    // strings with referencing functions
    try (PrintWriter w = new PrintWriter(new FileWriter(out + "/strings_xref.tsv"))) {
      for (Data s : currentProgram.getListing().getDefinedData(true)) {
        if (!s.hasStringValue()) continue;
        StringBuilder fs = new StringBuilder();
        for (Reference ref : getReferencesTo(s.getAddress())) {
          Function f = getFunctionContaining(ref.getFromAddress());
          if (f != null) fs.append(f.getName()).append(",");
        }
        String v = String.valueOf(s.getValue()).replace("\n","\\n").replace("\t","\\t");
        w.println(s.getAddress() + "\t" + fs + "\t" + v);
      }
    }
  }
}
