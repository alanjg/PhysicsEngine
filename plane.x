xof 0303txt 0032
template VertexDuplicationIndices {
 <b8d65549-d7c9-4995-89cf-53a9a8b031e3>
 DWORD nIndices;
 DWORD nOriginalVertices;
 array DWORD indices[nIndices];
}

template EffectParamDWord {
 <e13963bc-ae51-4c5d-b00f-cfa3a9d97ce5>
 STRING ParamName;
 DWORD Value;
}

template FVFData {
 <b6e70a0e-8ef9-4e83-94ad-ecc8b0c04897>
 DWORD dwFVF;
 DWORD nDWords;
 array DWORD data[nDWords];
}

template EffectFloats {
 <f1cfe2b3-0de3-4e28-afa1-155a750a282d>
 DWORD nFloats;
 array FLOAT Floats[nFloats];
}

template EffectString {
 <d55b097e-bdb6-4c52-b03d-6051c89d0e42>
 STRING Value;
}

template EffectDWord {
 <622c0ed0-956e-4da9-908a-2af94f3ce716>
 DWORD Value;
}

template EffectInstance {
 <e331f7e4-0559-4cc2-8e99-1cec1657928f>
 STRING EffectFilename;
 [...]
}

template AnimTicksPerSecond {
 <9e415a43-7ba6-4a73-8743-b73d47e88476>
 DWORD AnimTicksPerSecond;
}

template VertexElement {
 <f752461c-1e23-48f6-b9f8-8350850f336f>
 DWORD Type;
 DWORD Method;
 DWORD Usage;
 DWORD UsageIndex;
}

template DeclData {
 <bf22e553-292c-4781-9fea-62bd554bdd93>
 DWORD nElements;
 array VertexElement Elements[nElements];
 DWORD nDWords;
 array DWORD data[nDWords];
}

template EffectParamFloats {
 <3014b9a0-62f5-478c-9b86-e4ac9f4e418b>
 STRING ParamName;
 DWORD nFloats;
 array FLOAT Floats[nFloats];
}

template EffectParamString {
 <1dbc4c88-94c1-46ee-9076-2c28818c9481>
 STRING ParamName;
 STRING Value;
}


Frame {
 

 FrameTransformMatrix {
  1.000000,0.000000,0.000000,0.000000,0.000000,1.000000,0.000000,0.000000,0.000000,0.000000,1.000000,0.000000,0.000000,0.000000,0.000000,1.000000;;
 }

 Mesh {
 4;
 -10;0;-10;,
 -10;0;10;,
 10;0;10;,
 10;0;-10;;
  2;
  3;0,1,2;,
  3;0,2,3;;
  MeshNormals {
   4;
   0;1;0;,
   0;1;0;,
   0;1;0;,
   0;1;0;;
   2;
   3;0,1,2;,
   3;0,2,3;;  }

  MeshMaterialList {
   1;
   2;
   0,
   0;
      Material {
    1.000000;1.000000;1.000000;1.000000;;
    0.000000;
    1.000000;1.000000;1.000000;;
    0.000000;0.000000;0.000000;;
   }
  }

  VertexDuplicationIndices {
   0;
   0;
   
  }
 }
}