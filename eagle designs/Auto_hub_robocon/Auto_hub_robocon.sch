<?xml version="1.0" encoding="utf-8"?>
<!DOCTYPE eagle SYSTEM "eagle.dtd">
<eagle version="8.4.3">
<drawing>
<settings>
<setting alwaysvectorfont="no"/>
<setting verticaltext="up"/>
</settings>
<grid distance="0.1" unitdist="inch" unit="inch" style="lines" multiple="1" display="no" altdistance="0.01" altunitdist="inch" altunit="inch"/>
<layers>
<layer number="1" name="Top" color="4" fill="1" visible="no" active="no"/>
<layer number="2" name="Route2" color="1" fill="3" visible="no" active="no"/>
<layer number="3" name="Route3" color="4" fill="3" visible="no" active="no"/>
<layer number="4" name="Route4" color="1" fill="4" visible="no" active="no"/>
<layer number="5" name="Route5" color="4" fill="4" visible="no" active="no"/>
<layer number="6" name="Route6" color="1" fill="8" visible="no" active="no"/>
<layer number="7" name="Route7" color="4" fill="8" visible="no" active="no"/>
<layer number="8" name="Route8" color="1" fill="2" visible="no" active="no"/>
<layer number="9" name="Route9" color="4" fill="2" visible="no" active="no"/>
<layer number="10" name="Route10" color="1" fill="7" visible="no" active="no"/>
<layer number="11" name="Route11" color="4" fill="7" visible="no" active="no"/>
<layer number="12" name="Route12" color="1" fill="5" visible="no" active="no"/>
<layer number="13" name="Route13" color="4" fill="5" visible="no" active="no"/>
<layer number="14" name="Route14" color="1" fill="6" visible="no" active="no"/>
<layer number="15" name="Route15" color="4" fill="6" visible="no" active="no"/>
<layer number="16" name="Bottom" color="1" fill="1" visible="no" active="no"/>
<layer number="17" name="Pads" color="2" fill="1" visible="no" active="no"/>
<layer number="18" name="Vias" color="2" fill="1" visible="no" active="no"/>
<layer number="19" name="Unrouted" color="6" fill="1" visible="no" active="no"/>
<layer number="20" name="Dimension" color="24" fill="1" visible="no" active="no"/>
<layer number="21" name="tPlace" color="7" fill="1" visible="no" active="no"/>
<layer number="22" name="bPlace" color="7" fill="1" visible="no" active="no"/>
<layer number="23" name="tOrigins" color="15" fill="1" visible="no" active="no"/>
<layer number="24" name="bOrigins" color="15" fill="1" visible="no" active="no"/>
<layer number="25" name="tNames" color="7" fill="1" visible="no" active="no"/>
<layer number="26" name="bNames" color="7" fill="1" visible="no" active="no"/>
<layer number="27" name="tValues" color="7" fill="1" visible="no" active="no"/>
<layer number="28" name="bValues" color="7" fill="1" visible="no" active="no"/>
<layer number="29" name="tStop" color="7" fill="3" visible="no" active="no"/>
<layer number="30" name="bStop" color="7" fill="6" visible="no" active="no"/>
<layer number="31" name="tCream" color="7" fill="4" visible="no" active="no"/>
<layer number="32" name="bCream" color="7" fill="5" visible="no" active="no"/>
<layer number="33" name="tFinish" color="6" fill="3" visible="no" active="no"/>
<layer number="34" name="bFinish" color="6" fill="6" visible="no" active="no"/>
<layer number="35" name="tGlue" color="7" fill="4" visible="no" active="no"/>
<layer number="36" name="bGlue" color="7" fill="5" visible="no" active="no"/>
<layer number="37" name="tTest" color="7" fill="1" visible="no" active="no"/>
<layer number="38" name="bTest" color="7" fill="1" visible="no" active="no"/>
<layer number="39" name="tKeepout" color="4" fill="11" visible="no" active="no"/>
<layer number="40" name="bKeepout" color="1" fill="11" visible="no" active="no"/>
<layer number="41" name="tRestrict" color="4" fill="10" visible="no" active="no"/>
<layer number="42" name="bRestrict" color="1" fill="10" visible="no" active="no"/>
<layer number="43" name="vRestrict" color="2" fill="10" visible="no" active="no"/>
<layer number="44" name="Drills" color="7" fill="1" visible="no" active="no"/>
<layer number="45" name="Holes" color="7" fill="1" visible="no" active="no"/>
<layer number="46" name="Milling" color="3" fill="1" visible="no" active="no"/>
<layer number="47" name="Measures" color="7" fill="1" visible="no" active="no"/>
<layer number="48" name="Document" color="7" fill="1" visible="no" active="no"/>
<layer number="49" name="Reference" color="7" fill="1" visible="no" active="no"/>
<layer number="51" name="tDocu" color="7" fill="1" visible="no" active="no"/>
<layer number="52" name="bDocu" color="7" fill="1" visible="no" active="no"/>
<layer number="88" name="SimResults" color="9" fill="1" visible="yes" active="yes"/>
<layer number="89" name="SimProbes" color="9" fill="1" visible="yes" active="yes"/>
<layer number="90" name="Modules" color="5" fill="1" visible="yes" active="yes"/>
<layer number="91" name="Nets" color="2" fill="1" visible="yes" active="yes"/>
<layer number="92" name="Busses" color="1" fill="1" visible="yes" active="yes"/>
<layer number="93" name="Pins" color="2" fill="1" visible="no" active="yes"/>
<layer number="94" name="Symbols" color="4" fill="1" visible="yes" active="yes"/>
<layer number="95" name="Names" color="7" fill="1" visible="yes" active="yes"/>
<layer number="96" name="Values" color="7" fill="1" visible="yes" active="yes"/>
<layer number="97" name="Info" color="7" fill="1" visible="yes" active="yes"/>
<layer number="98" name="Guide" color="6" fill="1" visible="yes" active="yes"/>
</layers>
<schematic xreflabel="%F%N/%S.%C%R" xrefpart="/%S.%C%R">
<libraries>
<library name="Wurth_Elektronik_Electromechanic_Wire_to_Board_Connectors_rev17b" urn="urn:adsk.eagle:library:493">
<description>&lt;BR&gt;Wurth Elektronik - Wire to Board Connectors&lt;br&gt;&lt;Hr&gt;
&lt;BR&gt;&lt;BR&gt; 
&lt;TABLE BORDER=0 CELLSPACING=0 CELLPADDING=0&gt;
&lt;TR&gt;   
&lt;TD BGCOLOR="#cccccc" ALIGN=CENTER&gt;&lt;FONT FACE=ARIAL SIZE=3&gt;&lt;BR&gt;&lt;br&gt;
      &amp;nbsp;&amp;nbsp;&amp;nbsp;&amp;nbsp; &amp;nbsp;&lt;BR&gt;
       &lt;BR&gt;
       &lt;BR&gt;
       &lt;BR&gt;&lt;BR&gt;&lt;/FONT&gt;
    &lt;/TD&gt;
&lt;TD BGCOLOR="#cccccc" ALIGN=CENTER&gt;&lt;FONT FACE=ARIAL SIZE=3&gt;&lt;br&gt;
      -----&lt;BR&gt;
      -----&lt;BR&gt;
      -----&lt;BR&gt;
      -----&lt;BR&gt;
      -----&lt;BR&gt;&lt;BR&gt;&lt;/FONT&gt;
    &lt;/TD&gt;
    &lt;TD BGCOLOR="#cccccc" ALIGN=CENTER&gt; &lt;FONT FACE=ARIAL SIZE=3&gt;&lt;br&gt;
      ---------------------------&lt;BR&gt;
&lt;B&gt;&lt;I&gt;&lt;span style='font-size:26pt;
  color:#FF6600;'&gt;WE &lt;/span&gt;&lt;/i&gt;&lt;/b&gt;
&lt;BR&gt;
      ---------------------------&lt;BR&gt;&lt;b&gt;Würth Elektronik&lt;/b&gt;&lt;/FONT&gt;
    &lt;/TD&gt;
    &lt;TD BGCOLOR="#cccccc" ALIGN=CENTER&gt;&lt;FONT FACE=ARIAL SIZE=3&gt;&lt;br&gt;
      ---------O---&lt;BR&gt;
      ----O--------&lt;BR&gt;
      ---------O---&lt;BR&gt;
      ----O--------&lt;BR&gt;
      ---------O---&lt;BR&gt;&lt;BR&gt;&lt;/FONT&gt;
    &lt;/TD&gt;
   
&lt;TD BGCOLOR="#cccccc" ALIGN=CENTER&gt;&lt;FONT FACE=ARIAL SIZE=3&gt;&lt;BR&gt;
      &amp;nbsp;&amp;nbsp;&amp;nbsp;&amp;nbsp; &amp;nbsp;&lt;BR&gt;
       &lt;BR&gt;
       &lt;BR&gt;
       &lt;BR&gt;
       &lt;BR&gt;&lt;BR&gt;&lt;/FONT&gt;
    &lt;/TD&gt;
  &lt;/TR&gt;

  &lt;TR&gt;
    &lt;TD COLSPAN=7&gt;&amp;nbsp;
    &lt;/TD&gt;
  &lt;/TR&gt;
  
&lt;/TABLE&gt;
&lt;B&gt;&amp;nbsp;&amp;nbsp;&amp;nbsp;&amp;nbsp;&amp;nbsp;&amp;nbsp;&amp;nbsp;&amp;nbsp;&amp;nbsp;&amp;nbsp;&amp;nbsp;&amp;nbsp;&amp;nbsp;&amp;nbsp;&amp;nbsp;&amp;nbsp;&amp;nbsp;&amp;nbsp;&amp;nbsp;&amp;nbsp;&amp;nbsp;&amp;nbsp;&amp;nbsp;&amp;nbsp;&amp;nbsp;&amp;nbsp;&amp;nbsp;&amp;nbsp;&amp;nbsp;&amp;nbsp;&amp;nbsp;&amp;nbsp;&amp;nbsp;&amp;nbsp;&amp;nbsp;&amp;nbsp;&amp;nbsp;&amp;nbsp;&amp;nbsp;More than you expect&lt;BR&gt;&lt;BR&gt;&lt;BR&gt;&lt;/B&gt;

&lt;HR&gt;&lt;BR&gt;
&lt;b&gt;Würth Elektronik eiSos GmbH &amp; Co. KG&lt;/b&gt;&lt;br&gt;
EMC &amp; Inductive Solutions&lt;br&gt;

Max-Eyth-Str.1&lt;br&gt;
D-74638 Waldenburg&lt;br&gt;
&lt;br&gt;
Tel: +49 (0)7942-945-0&lt;br&gt;
Fax:+49 (0)7942-945-405&lt;br&gt;
&lt;br&gt;
&lt;a href="http://www.we-online.com/web/en/electronic_components/produkte_pb/bauteilebibliotheken/eagle_4.php"&gt;www.we-online.com/eagle&lt;/a&gt;&lt;br&gt;
&lt;a href="mailto:libraries@we-online.com"&gt;libraries@we-online.com&lt;/a&gt; &lt;BR&gt;&lt;BR&gt;
&lt;br&gt;&lt;HR&gt;&lt;BR&gt;
Neither Autodesk nor Würth Elektronik eiSos does warrant that this library is error-free or &lt;br&gt;
that it meets your specific requirements.&lt;br&gt;&lt;BR&gt;
Please contact us for more information.&lt;br&gt;&lt;BR&gt;&lt;br&gt;
&lt;hr&gt;
Eagle Version 6, Library Revision 2017b, 2017-06-14&lt;br&gt;
&lt;HR&gt;
Copyright: Würth Elektronik</description>
<packages>
<package name="61202421621" library_version="1">
<description>WR-BHD 2.54mm Male Box Header, 24 Pins</description>
<wire x1="-19.07" y1="4.55" x2="19.07" y2="4.55" width="0.127" layer="21"/>
<wire x1="19.07" y1="4.55" x2="19.07" y2="-4.55" width="0.127" layer="21"/>
<wire x1="19.07" y1="-4.55" x2="-19.07" y2="-4.55" width="0.127" layer="21"/>
<wire x1="-19.07" y1="-4.55" x2="-19.07" y2="4.55" width="0.127" layer="21"/>
<pad name="2" x="-13.97" y="1.27" drill="1.1"/>
<pad name="1" x="-13.97" y="-1.27" drill="1.1"/>
<pad name="4" x="-11.43" y="1.27" drill="1.1"/>
<pad name="3" x="-11.43" y="-1.27" drill="1.1"/>
<pad name="6" x="-8.89" y="1.27" drill="1.1"/>
<pad name="5" x="-8.89" y="-1.27" drill="1.1"/>
<pad name="7" x="-6.35" y="-1.27" drill="1.1"/>
<pad name="8" x="-6.35" y="1.27" drill="1.1"/>
<pad name="9" x="-3.81" y="-1.27" drill="1.1"/>
<pad name="10" x="-3.81" y="1.27" drill="1.1"/>
<pad name="11" x="-1.27" y="-1.27" drill="1.1"/>
<pad name="12" x="-1.27" y="1.27" drill="1.1"/>
<pad name="13" x="1.27" y="-1.27" drill="1.1"/>
<pad name="14" x="1.27" y="1.27" drill="1.1"/>
<pad name="15" x="3.81" y="-1.27" drill="1.1"/>
<pad name="16" x="3.81" y="1.27" drill="1.1"/>
<pad name="17" x="6.35" y="-1.27" drill="1.1"/>
<pad name="18" x="6.35" y="1.27" drill="1.1"/>
<pad name="19" x="8.89" y="-1.27" drill="1.1"/>
<pad name="20" x="8.89" y="1.27" drill="1.1"/>
<pad name="21" x="11.43" y="-1.27" drill="1.1"/>
<pad name="22" x="11.43" y="1.27" drill="1.1"/>
<pad name="23" x="13.97" y="-1.27" drill="1.1"/>
<pad name="24" x="13.97" y="1.27" drill="1.1"/>
<text x="-15.875" y="-1.905" size="1.27" layer="51">1</text>
<text x="15.4" y="0.535" size="1.27" layer="51">24</text>
<text x="-3.175" y="6.35" size="1.27" layer="25">&gt;NAME</text>
<text x="-2.54" y="-7.62" size="1.27" layer="27">&gt;VALUE</text>
<text x="-15.875" y="-1.905" size="1.27" layer="21">1</text>
<polygon width="0.127" layer="39">
<vertex x="-19.32" y="4.8"/>
<vertex x="19.32" y="4.8"/>
<vertex x="19.32" y="-4.8"/>
<vertex x="-19.32" y="-4.8"/>
</polygon>
</package>
<package name="61201421621" library_version="1">
<description>WR-BHD 2.54mm Male Box Header, 14 Pins</description>
<wire x1="-12.72" y1="4.55" x2="12.72" y2="4.55" width="0.127" layer="21"/>
<wire x1="12.72" y1="4.55" x2="12.72" y2="-4.55" width="0.127" layer="21"/>
<wire x1="12.72" y1="-4.55" x2="-12.72" y2="-4.55" width="0.127" layer="21"/>
<wire x1="-12.72" y1="-4.55" x2="-12.72" y2="4.55" width="0.127" layer="21"/>
<pad name="2" x="-7.62" y="1.27" drill="1.1"/>
<pad name="1" x="-7.62" y="-1.27" drill="1.1"/>
<pad name="4" x="-5.08" y="1.27" drill="1.1"/>
<pad name="3" x="-5.08" y="-1.27" drill="1.1"/>
<pad name="6" x="-2.54" y="1.27" drill="1.1"/>
<pad name="5" x="-2.54" y="-1.27" drill="1.1"/>
<pad name="7" x="0" y="-1.27" drill="1.1"/>
<pad name="8" x="0" y="1.27" drill="1.1"/>
<pad name="9" x="2.54" y="-1.27" drill="1.1"/>
<pad name="10" x="2.54" y="1.27" drill="1.1"/>
<pad name="11" x="5.08" y="-1.27" drill="1.1"/>
<pad name="12" x="5.08" y="1.27" drill="1.1"/>
<pad name="13" x="7.62" y="-1.27" drill="1.1"/>
<pad name="14" x="7.62" y="1.27" drill="1.1"/>
<text x="-9.525" y="-1.905" size="1.27" layer="51">1</text>
<text x="9.05" y="0.535" size="1.27" layer="51">14</text>
<text x="-3.175" y="6.35" size="1.27" layer="25">&gt;NAME</text>
<text x="-3.81" y="-6.35" size="1.27" layer="27">&gt;VALUE</text>
<text x="-9.525" y="-1.905" size="1.27" layer="21">1</text>
<polygon width="0.127" layer="39">
<vertex x="-12.97" y="4.8"/>
<vertex x="12.97" y="4.8"/>
<vertex x="12.97" y="-4.8"/>
<vertex x="-12.97" y="-4.8"/>
</polygon>
</package>
</packages>
<symbols>
<symbol name="12X12" library_version="1">
<wire x1="-25.4" y1="5.08" x2="-25.4" y2="-5.08" width="0.254" layer="94"/>
<wire x1="35.56" y1="-5.08" x2="35.56" y2="5.08" width="0.254" layer="94"/>
<wire x1="35.56" y1="5.08" x2="-25.4" y2="5.08" width="0.254" layer="94"/>
<wire x1="35.56" y1="-5.08" x2="-25.4" y2="-5.08" width="0.254" layer="94"/>
<text x="-35.052" y="0.762" size="1.778" layer="95">&gt;NAME</text>
<text x="-35.814" y="-2.54" size="1.778" layer="96">&gt;VALUE</text>
<pin name="1" x="-22.86" y="-10.16" visible="pad" length="middle" direction="pas" rot="R90"/>
<pin name="2" x="-22.86" y="10.16" visible="pad" length="middle" direction="pas" rot="R270"/>
<pin name="3" x="-17.78" y="-10.16" visible="pad" length="middle" direction="pas" rot="R90"/>
<pin name="4" x="-17.78" y="10.16" visible="pad" length="middle" direction="pas" rot="R270"/>
<pin name="5" x="-12.7" y="-10.16" visible="pad" length="middle" direction="pas" rot="R90"/>
<pin name="6" x="-12.7" y="10.16" visible="pad" length="middle" direction="pas" rot="R270"/>
<pin name="7" x="-7.62" y="-10.16" visible="pad" length="middle" direction="pas" rot="R90"/>
<pin name="8" x="-7.62" y="10.16" visible="pad" length="middle" direction="pas" rot="R270"/>
<pin name="10" x="-2.54" y="10.16" visible="pad" length="middle" direction="pas" rot="R270"/>
<pin name="9" x="-2.54" y="-10.16" visible="pad" length="middle" direction="pas" rot="R90"/>
<pin name="12" x="2.54" y="10.16" visible="pad" length="middle" direction="pas" rot="R270"/>
<pin name="11" x="2.54" y="-10.16" visible="pad" length="middle" direction="pas" rot="R90"/>
<pin name="14" x="7.62" y="10.16" visible="pad" length="middle" direction="pas" rot="R270"/>
<pin name="13" x="7.62" y="-10.16" visible="pad" length="middle" direction="pas" rot="R90"/>
<pin name="15" x="12.7" y="-10.16" visible="pad" length="middle" direction="pas" rot="R90"/>
<pin name="16" x="12.7" y="10.16" visible="pad" length="middle" direction="pas" rot="R270"/>
<pin name="17" x="17.78" y="-10.16" visible="pad" length="middle" direction="pas" rot="R90"/>
<pin name="18" x="17.78" y="10.16" visible="pad" length="middle" direction="pas" rot="R270"/>
<pin name="19" x="22.86" y="-10.16" visible="pad" length="middle" direction="pas" rot="R90"/>
<pin name="20" x="22.86" y="10.16" visible="pad" length="middle" direction="pas" rot="R270"/>
<pin name="21" x="27.94" y="-10.16" visible="pad" length="middle" direction="pas" rot="R90"/>
<pin name="22" x="27.94" y="10.16" visible="pad" length="middle" direction="pas" rot="R270"/>
<pin name="23" x="33.02" y="-10.16" visible="pad" length="middle" direction="pas" rot="R90"/>
<pin name="24" x="33.02" y="10.16" visible="pad" length="middle" direction="pas" rot="R270"/>
</symbol>
<symbol name="7X7" library_version="1">
<wire x1="-12.7" y1="5.08" x2="-12.7" y2="-5.08" width="0.254" layer="94"/>
<wire x1="-12.7" y1="-5.08" x2="22.86" y2="-5.08" width="0.254" layer="94"/>
<wire x1="22.86" y1="-5.08" x2="22.86" y2="5.08" width="0.254" layer="94"/>
<wire x1="22.86" y1="5.08" x2="-12.7" y2="5.08" width="0.254" layer="94"/>
<text x="-22.352" y="0.762" size="1.778" layer="95">&gt;NAME</text>
<text x="-23.114" y="-2.54" size="1.778" layer="96">&gt;VALUE</text>
<pin name="1" x="-10.16" y="-10.16" visible="pad" length="middle" direction="pas" rot="R90"/>
<pin name="2" x="-10.16" y="10.16" visible="pad" length="middle" direction="pas" rot="R270"/>
<pin name="3" x="-5.08" y="-10.16" visible="pad" length="middle" direction="pas" rot="R90"/>
<pin name="4" x="-5.08" y="10.16" visible="pad" length="middle" direction="pas" rot="R270"/>
<pin name="5" x="0" y="-10.16" visible="pad" length="middle" direction="pas" rot="R90"/>
<pin name="6" x="0" y="10.16" visible="pad" length="middle" direction="pas" rot="R270"/>
<pin name="7" x="5.08" y="-10.16" visible="pad" length="middle" direction="pas" rot="R90"/>
<pin name="8" x="5.08" y="10.16" visible="pad" length="middle" direction="pas" rot="R270"/>
<pin name="10" x="10.16" y="10.16" visible="pad" length="middle" direction="pas" rot="R270"/>
<pin name="9" x="10.16" y="-10.16" visible="pad" length="middle" direction="pas" rot="R90"/>
<pin name="12" x="15.24" y="10.16" visible="pad" length="middle" direction="pas" rot="R270"/>
<pin name="11" x="15.24" y="-10.16" visible="pad" length="middle" direction="pas" rot="R90"/>
<pin name="14" x="20.32" y="10.16" visible="pad" length="middle" direction="pas" rot="R270"/>
<pin name="13" x="20.32" y="-10.16" visible="pad" length="middle" direction="pas" rot="R90"/>
</symbol>
</symbols>
<devicesets>
<deviceset name="61202421621" prefix="J" uservalue="yes" library_version="1">
<description>&lt;b&gt; WR-BHD 2.54mm Male Box Header, 24 Pins&lt;/b&gt;=&gt;Code : Con_WTB_BHD_2.54_Male_THT_WS_61202421621
&lt;br&gt;&lt;a href="http://katalog.we-online.de/media/thumbs2/eican/thb_Con_WTB_BHD_2.54_Male_THT_WS_6120xx21621_pf2.jpg" title="Enlarge picture"&gt;
&lt;img src="http://katalog.we-online.de/media/thumbs2/eican/thb_Con_WTB_BHD_2.54_Male_THT_WS_6120xx21621_pf2.jpg"  width="320"&gt;&lt;/a&gt;&lt;p&gt;
Details see: &lt;a href="http://katalog.we-online.de/en/em/612_0xx_216_21"&gt;http://katalog.we-online.de/en/em/612_0xx_216_21&lt;/a&gt;&lt;p&gt;
Created 2014-06-04, Karrer Zheng&lt;br&gt;
2014 (C) Würth Elektronik</description>
<gates>
<gate name="G$1" symbol="12X12" x="0" y="0"/>
</gates>
<devices>
<device name="" package="61202421621">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="10" pad="10"/>
<connect gate="G$1" pin="11" pad="11"/>
<connect gate="G$1" pin="12" pad="12"/>
<connect gate="G$1" pin="13" pad="13"/>
<connect gate="G$1" pin="14" pad="14"/>
<connect gate="G$1" pin="15" pad="15"/>
<connect gate="G$1" pin="16" pad="16"/>
<connect gate="G$1" pin="17" pad="17"/>
<connect gate="G$1" pin="18" pad="18"/>
<connect gate="G$1" pin="19" pad="19"/>
<connect gate="G$1" pin="2" pad="2"/>
<connect gate="G$1" pin="20" pad="20"/>
<connect gate="G$1" pin="21" pad="21"/>
<connect gate="G$1" pin="22" pad="22"/>
<connect gate="G$1" pin="23" pad="23"/>
<connect gate="G$1" pin="24" pad="24"/>
<connect gate="G$1" pin="3" pad="3"/>
<connect gate="G$1" pin="4" pad="4"/>
<connect gate="G$1" pin="5" pad="5"/>
<connect gate="G$1" pin="6" pad="6"/>
<connect gate="G$1" pin="7" pad="7"/>
<connect gate="G$1" pin="8" pad="8"/>
<connect gate="G$1" pin="9" pad="9"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
</devices>
</deviceset>
<deviceset name="61201421621" prefix="J" uservalue="yes" library_version="1">
<description>&lt;b&gt; WR-BHD 2.54mm Male Box Header, 14 Pins&lt;/b&gt;=&gt;Code : Con_WTB_BHD_2.54_Male_THT_WS_61201421621
&lt;br&gt;&lt;a href="http://katalog.we-online.de/media/thumbs2/eican/thb_Con_WTB_BHD_2.54_Male_THT_WS_6120xx21621_pf2.jpg" title="Enlarge picture"&gt;
&lt;img src="http://katalog.we-online.de/media/thumbs2/eican/thb_Con_WTB_BHD_2.54_Male_THT_WS_6120xx21621_pf2.jpg"  width="320"&gt;&lt;/a&gt;&lt;p&gt;
Details see: &lt;a href="http://katalog.we-online.de/en/em/612_0xx_216_21"&gt;http://katalog.we-online.de/en/em/612_0xx_216_21&lt;/a&gt;&lt;p&gt;
Created 2014-06-04, Karrer Zheng&lt;br&gt;
2014 (C) Würth Elektronik</description>
<gates>
<gate name="G$1" symbol="7X7" x="0" y="0"/>
</gates>
<devices>
<device name="" package="61201421621">
<connects>
<connect gate="G$1" pin="1" pad="1"/>
<connect gate="G$1" pin="10" pad="10"/>
<connect gate="G$1" pin="11" pad="11"/>
<connect gate="G$1" pin="12" pad="12"/>
<connect gate="G$1" pin="13" pad="13"/>
<connect gate="G$1" pin="14" pad="14"/>
<connect gate="G$1" pin="2" pad="2"/>
<connect gate="G$1" pin="3" pad="3"/>
<connect gate="G$1" pin="4" pad="4"/>
<connect gate="G$1" pin="5" pad="5"/>
<connect gate="G$1" pin="6" pad="6"/>
<connect gate="G$1" pin="7" pad="7"/>
<connect gate="G$1" pin="8" pad="8"/>
<connect gate="G$1" pin="9" pad="9"/>
</connects>
<technologies>
<technology name=""/>
</technologies>
</device>
</devices>
</deviceset>
</devicesets>
</library>
<library name="uln-udn" urn="urn:adsk.eagle:library:407">
<description>&lt;b&gt;Driver Arrays&lt;/b&gt;&lt;p&gt;
ULN and UDN Series&lt;p&gt;
&lt;author&gt;Created by librarian@cadsoft.de&lt;/author&gt;</description>
<packages>
<package name="DIL16" urn="urn:adsk.eagle:footprint:30226/1" library_version="1">
<description>&lt;b&gt;Dual In Line Package&lt;/b&gt;</description>
<wire x1="10.16" y1="2.921" x2="-10.16" y2="2.921" width="0.1524" layer="21"/>
<wire x1="-10.16" y1="-2.921" x2="10.16" y2="-2.921" width="0.1524" layer="21"/>
<wire x1="10.16" y1="2.921" x2="10.16" y2="-2.921" width="0.1524" layer="21"/>
<wire x1="-10.16" y1="2.921" x2="-10.16" y2="1.016" width="0.1524" layer="21"/>
<wire x1="-10.16" y1="-2.921" x2="-10.16" y2="-1.016" width="0.1524" layer="21"/>
<wire x1="-10.16" y1="1.016" x2="-10.16" y2="-1.016" width="0.1524" layer="21" curve="-180"/>
<pad name="1" x="-8.89" y="-3.81" drill="0.8128" shape="long" rot="R90"/>
<pad name="2" x="-6.35" y="-3.81" drill="0.8128" shape="long" rot="R90"/>
<pad name="7" x="6.35" y="-3.81" drill="0.8128" shape="long" rot="R90"/>
<pad name="8" x="8.89" y="-3.81" drill="0.8128" shape="long" rot="R90"/>
<pad name="3" x="-3.81" y="-3.81" drill="0.8128" shape="long" rot="R90"/>
<pad name="4" x="-1.27" y="-3.81" drill="0.8128" shape="long" rot="R90"/>
<pad name="6" x="3.81" y="-3.81" drill="0.8128" shape="long" rot="R90"/>
<pad name="5" x="1.27" y="-3.81" drill="0.8128" shape="long" rot="R90"/>
<pad name="9" x="8.89" y="3.81" drill="0.8128" shape="long" rot="R90"/>
<pad name="10" x="6.35" y="3.81" drill="0.8128" shape="long" rot="R90"/>
<pad name="11" x="3.81" y="3.81" drill="0.8128" shape="long" rot="R90"/>
<pad name="12" x="1.27" y="3.81" drill="0.8128" shape="long" rot="R90"/>
<pad name="13" x="-1.27" y="3.81" drill="0.8128" shape="long" rot="R90"/>
<pad name="14" x="-3.81" y="3.81" drill="0.8128" shape="long" rot="R90"/>
<pad name="15" x="-6.35" y="3.81" drill="0.8128" shape="long" rot="R90"/>
<pad name="16" x="-8.89" y="3.81" drill="0.8128" shape="long" rot="R90"/>
<text x="-10.541" y="-2.921" size="1.27" layer="25" ratio="10" rot="R90">&gt;NAME</text>
<text x="-7.493" y="-0.635" size="1.27" layer="27" ratio="10">&gt;VALUE</text>
</package>
<package name="SO16" urn="urn:adsk.eagle:footprint:30227/1" library_version="1">
<description>&lt;b&gt;Small Outline Package&lt;/b&gt;</description>
<wire x1="4.699" y1="1.9558" x2="-4.699" y2="1.9558" width="0.1524" layer="21"/>
<wire x1="4.699" y1="-1.9558" x2="5.08" y2="-1.5748" width="0.1524" layer="21" curve="90"/>
<wire x1="-5.08" y1="1.5748" x2="-4.699" y2="1.9558" width="0.1524" layer="21" curve="-90"/>
<wire x1="4.699" y1="1.9558" x2="5.08" y2="1.5748" width="0.1524" layer="21" curve="-90"/>
<wire x1="-5.08" y1="-1.5748" x2="-4.699" y2="-1.9558" width="0.1524" layer="21" curve="90"/>
<wire x1="-4.699" y1="-1.9558" x2="4.699" y2="-1.9558" width="0.1524" layer="21"/>
<wire x1="5.08" y1="-1.5748" x2="5.08" y2="1.5748" width="0.1524" layer="21"/>
<wire x1="-5.08" y1="1.5748" x2="-5.08" y2="-1.5748" width="0.1524" layer="21"/>
<wire x1="-5.08" y1="0.508" x2="-5.08" y2="-0.508" width="0.1524" layer="21" curve="-180"/>
<wire x1="-5.08" y1="-1.6002" x2="5.08" y2="-1.6002" width="0.0508" layer="21"/>
<smd name="1" x="-4.445" y="-3.0734" dx="0.6604" dy="2.032" layer="1"/>
<smd name="16" x="-4.445" y="3.0734" dx="0.6604" dy="2.032" layer="1"/>
<smd name="2" x="-3.175" y="-3.0734" dx="0.6604" dy="2.032" layer="1"/>
<smd name="3" x="-1.905" y="-3.0734" dx="0.6604" dy="2.032" layer="1"/>
<smd name="15" x="-3.175" y="3.0734" dx="0.6604" dy="2.032" layer="1"/>
<smd name="14" x="-1.905" y="3.0734" dx="0.6604" dy="2.032" layer="1"/>
<smd name="4" x="-0.635" y="-3.0734" dx="0.6604" dy="2.032" layer="1"/>
<smd name="13" x="-0.635" y="3.0734" dx="0.6604" dy="2.032" layer="1"/>
<smd name="5" x="0.635" y="-3.0734" dx="0.6604" dy="2.032" layer="1"/>
<smd name="12" x="0.635" y="3.0734" dx="0.6604" dy="2.032" layer="1"/>
<smd name="6" x="1.905" y="-3.0734" dx="0.6604" dy="2.032" layer="1"/>
<smd name="7" x="3.175" y="-3.0734" dx="0.6604" dy="2.032" layer="1"/>
<smd name="11" x="1.905" y="3.0734" dx="0.6604" dy="2.032" layer="1"/>
<smd name="10" x="3.175" y="3.0734" dx="0.6604" dy="2.032" layer="1"/>
<smd name="8" x="4.445" y="-3.0734" dx="0.6604" dy="2.032" layer="1"/>
<smd name="9" x="4.445" y="3.0734" dx="0.6604" dy="2.032" layer="1"/>
<text x="-4.064" y="-0.635" size="1.27" layer="27" ratio="10">&gt;VALUE</text>
<text x="-5.461" y="-1.778" size="1.27" layer="25" ratio="10" rot="R90">&gt;NAME</text>
<rectangle x1="-0.889" y1="1.9558" x2="-0.381" y2="3.0988" layer="51"/>
<rectangle x1="-4.699" y1="-3.0988" x2="-4.191" y2="-1.9558" layer="51"/>
<rectangle x1="-3.429" y1="-3.0988" x2="-2.921" y2="-1.9558" layer="51"/>
<rectangle x1="-2.159" y1="-3.0734" x2="-1.651" y2="-1.9304" layer="51"/>
<rectangle x1="-0.889" y1="-3.0988" x2="-0.381" y2="-1.9558" layer="51"/>
<rectangle x1="-2.159" y1="1.9558" x2="-1.651" y2="3.0988" layer="51"/>
<rectangle x1="-3.429" y1="1.9558" x2="-2.921" y2="3.0988" layer="51"/>
<rectangle x1="-4.699" y1="1.9558" x2="-4.191" y2="3.0988" layer="51"/>
<rectangle x1="0.381" y1="-3.0988" x2="0.889" y2="-1.9558" layer="51"/>
<rectangle x1="1.651" y1="-3.0988" x2="2.159" y2="-1.9558" layer="51"/>
<rectangle x1="2.921" y1="-3.0988" x2="3.429" y2="-1.9558" layer="51"/>
<rectangle x1="4.191" y1="-3.0988" x2="4.699" y2="-1.9558" layer="51"/>
<rectangle x1="0.381" y1="1.9558" x2="0.889" y2="3.0988" layer="51"/>
<rectangle x1="1.651" y1="1.9558" x2="2.159" y2="3.0988" layer="51"/>
<rectangle x1="2.921" y1="1.9558" x2="3.429" y2="3.0988" layer="51"/>
<rectangle x1="4.191" y1="1.9558" x2="4.699" y2="3.0988" layer="51"/>
</package>
</packages>
<packages3d>
<package3d name="DIL16" urn="urn:adsk.eagle:package:30239/1" type="box" library_version="1">
<description>Dual In Line Package</description>
</package3d>
<package3d name="SO16" urn="urn:adsk.eagle:package:30247/1" type="box" library_version="1">
<description>Small Outline Package</description>
</package3d>
</packages3d>
<symbols>
<symbol name="2001A" urn="urn:adsk.eagle:symbol:30225/1" library_version="1">
<wire x1="-7.62" y1="10.16" x2="7.62" y2="10.16" width="0.4064" layer="94"/>
<wire x1="7.62" y1="-12.7" x2="7.62" y2="10.16" width="0.4064" layer="94"/>
<wire x1="7.62" y1="-12.7" x2="-7.62" y2="-12.7" width="0.4064" layer="94"/>
<wire x1="-7.62" y1="10.16" x2="-7.62" y2="-12.7" width="0.4064" layer="94"/>
<text x="-7.62" y="10.922" size="1.778" layer="95">&gt;NAME</text>
<text x="-7.62" y="-15.24" size="1.778" layer="96">&gt;VALUE</text>
<pin name="I1" x="-12.7" y="7.62" length="middle" direction="in"/>
<pin name="I2" x="-12.7" y="5.08" length="middle" direction="in"/>
<pin name="I3" x="-12.7" y="2.54" length="middle" direction="in"/>
<pin name="I4" x="-12.7" y="0" length="middle" direction="in"/>
<pin name="I5" x="-12.7" y="-2.54" length="middle" direction="in"/>
<pin name="I6" x="-12.7" y="-5.08" length="middle" direction="in"/>
<pin name="I7" x="-12.7" y="-7.62" length="middle" direction="in"/>
<pin name="O1" x="12.7" y="7.62" length="middle" direction="oc" rot="R180"/>
<pin name="O2" x="12.7" y="5.08" length="middle" direction="oc" rot="R180"/>
<pin name="O3" x="12.7" y="2.54" length="middle" direction="oc" rot="R180"/>
<pin name="O4" x="12.7" y="0" length="middle" direction="oc" rot="R180"/>
<pin name="O5" x="12.7" y="-2.54" length="middle" direction="oc" rot="R180"/>
<pin name="O6" x="12.7" y="-5.08" length="middle" direction="oc" rot="R180"/>
<pin name="O7" x="12.7" y="-7.62" length="middle" direction="oc" rot="R180"/>
<pin name="CD+" x="12.7" y="-10.16" length="middle" direction="pas" rot="R180"/>
<pin name="GND" x="-12.7" y="-10.16" length="middle" direction="pwr"/>
</symbol>
</symbols>
<devicesets>
<deviceset name="ULN2003A" urn="urn:adsk.eagle:component:30250/1" prefix="IC" library_version="1">
<description>&lt;b&gt;DRIVER ARRAY&lt;/b&gt;</description>
<gates>
<gate name="A" symbol="2001A" x="0" y="0"/>
</gates>
<devices>
<device name="N" package="DIL16">
<connects>
<connect gate="A" pin="CD+" pad="9"/>
<connect gate="A" pin="GND" pad="8"/>
<connect gate="A" pin="I1" pad="1"/>
<connect gate="A" pin="I2" pad="2"/>
<connect gate="A" pin="I3" pad="3"/>
<connect gate="A" pin="I4" pad="4"/>
<connect gate="A" pin="I5" pad="5"/>
<connect gate="A" pin="I6" pad="6"/>
<connect gate="A" pin="I7" pad="7"/>
<connect gate="A" pin="O1" pad="16"/>
<connect gate="A" pin="O2" pad="15"/>
<connect gate="A" pin="O3" pad="14"/>
<connect gate="A" pin="O4" pad="13"/>
<connect gate="A" pin="O5" pad="12"/>
<connect gate="A" pin="O6" pad="11"/>
<connect gate="A" pin="O7" pad="10"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:30239/1"/>
</package3dinstances>
<technologies>
<technology name=""/>
</technologies>
</device>
<device name="D" package="SO16">
<connects>
<connect gate="A" pin="CD+" pad="9"/>
<connect gate="A" pin="GND" pad="8"/>
<connect gate="A" pin="I1" pad="1"/>
<connect gate="A" pin="I2" pad="2"/>
<connect gate="A" pin="I3" pad="3"/>
<connect gate="A" pin="I4" pad="4"/>
<connect gate="A" pin="I5" pad="5"/>
<connect gate="A" pin="I6" pad="6"/>
<connect gate="A" pin="I7" pad="7"/>
<connect gate="A" pin="O1" pad="16"/>
<connect gate="A" pin="O2" pad="15"/>
<connect gate="A" pin="O3" pad="14"/>
<connect gate="A" pin="O4" pad="13"/>
<connect gate="A" pin="O5" pad="12"/>
<connect gate="A" pin="O6" pad="11"/>
<connect gate="A" pin="O7" pad="10"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:30247/1"/>
</package3dinstances>
<technologies>
<technology name=""/>
</technologies>
</device>
</devices>
</deviceset>
</devicesets>
</library>
<library name="74xx-eu" urn="urn:adsk.eagle:library:85">
<description>&lt;b&gt;TTL Devices, 74xx Series with European Symbols&lt;/b&gt;&lt;p&gt;
Based on the following sources:
&lt;ul&gt;
&lt;li&gt;Texas Instruments &lt;i&gt;TTL Data Book&lt;/i&gt;&amp;nbsp;&amp;nbsp;&amp;nbsp;Volume 1, 1996.
&lt;li&gt;TTL Data Book, Volume 2 , 1993
&lt;li&gt;National Seminconductor Databook 1990, ALS/LS Logic
&lt;li&gt;ttl 74er digital data dictionary, ECA Electronic + Acustic GmbH, ISBN 3-88109-032-0
&lt;li&gt;http://icmaster.com/ViewCompare.asp
&lt;/ul&gt;
&lt;author&gt;Created by librarian@cadsoft.de&lt;/author&gt;</description>
<packages>
<package name="DIL14" urn="urn:adsk.eagle:footprint:1631/1" library_version="1">
<description>&lt;b&gt;Dual In Line Package&lt;/b&gt;</description>
<wire x1="8.89" y1="2.921" x2="-8.89" y2="2.921" width="0.1524" layer="21"/>
<wire x1="-8.89" y1="-2.921" x2="8.89" y2="-2.921" width="0.1524" layer="21"/>
<wire x1="8.89" y1="2.921" x2="8.89" y2="-2.921" width="0.1524" layer="21"/>
<wire x1="-8.89" y1="2.921" x2="-8.89" y2="1.016" width="0.1524" layer="21"/>
<wire x1="-8.89" y1="-2.921" x2="-8.89" y2="-1.016" width="0.1524" layer="21"/>
<wire x1="-8.89" y1="1.016" x2="-8.89" y2="-1.016" width="0.1524" layer="21" curve="-180"/>
<pad name="1" x="-7.62" y="-3.81" drill="0.8128" shape="long" rot="R90"/>
<pad name="2" x="-5.08" y="-3.81" drill="0.8128" shape="long" rot="R90"/>
<pad name="7" x="7.62" y="-3.81" drill="0.8128" shape="long" rot="R90"/>
<pad name="8" x="7.62" y="3.81" drill="0.8128" shape="long" rot="R90"/>
<pad name="3" x="-2.54" y="-3.81" drill="0.8128" shape="long" rot="R90"/>
<pad name="4" x="0" y="-3.81" drill="0.8128" shape="long" rot="R90"/>
<pad name="6" x="5.08" y="-3.81" drill="0.8128" shape="long" rot="R90"/>
<pad name="5" x="2.54" y="-3.81" drill="0.8128" shape="long" rot="R90"/>
<pad name="9" x="5.08" y="3.81" drill="0.8128" shape="long" rot="R90"/>
<pad name="10" x="2.54" y="3.81" drill="0.8128" shape="long" rot="R90"/>
<pad name="11" x="0" y="3.81" drill="0.8128" shape="long" rot="R90"/>
<pad name="12" x="-2.54" y="3.81" drill="0.8128" shape="long" rot="R90"/>
<pad name="13" x="-5.08" y="3.81" drill="0.8128" shape="long" rot="R90"/>
<pad name="14" x="-7.62" y="3.81" drill="0.8128" shape="long" rot="R90"/>
<text x="-9.271" y="-3.048" size="1.27" layer="25" ratio="10" rot="R90">&gt;NAME</text>
<text x="-6.731" y="-0.635" size="1.27" layer="27" ratio="10">&gt;VALUE</text>
</package>
<package name="SO14" urn="urn:adsk.eagle:footprint:1630/1" library_version="1">
<description>&lt;b&gt;Small Outline package&lt;/b&gt; 150 mil</description>
<wire x1="4.064" y1="1.9558" x2="-4.064" y2="1.9558" width="0.1524" layer="51"/>
<wire x1="4.064" y1="-1.9558" x2="4.445" y2="-1.5748" width="0.1524" layer="21" curve="90"/>
<wire x1="-4.445" y1="1.5748" x2="-4.064" y2="1.9558" width="0.1524" layer="21" curve="-90"/>
<wire x1="4.064" y1="1.9558" x2="4.445" y2="1.5748" width="0.1524" layer="21" curve="-90"/>
<wire x1="-4.445" y1="-1.5748" x2="-4.064" y2="-1.9558" width="0.1524" layer="21" curve="90"/>
<wire x1="-4.064" y1="-1.9558" x2="4.064" y2="-1.9558" width="0.1524" layer="51"/>
<wire x1="4.445" y1="-1.5748" x2="4.445" y2="1.5748" width="0.1524" layer="21"/>
<wire x1="-4.445" y1="1.5748" x2="-4.445" y2="0.508" width="0.1524" layer="21"/>
<wire x1="-4.445" y1="0.508" x2="-4.445" y2="-0.508" width="0.1524" layer="21"/>
<wire x1="-4.445" y1="-0.508" x2="-4.445" y2="-1.5748" width="0.1524" layer="21"/>
<wire x1="-4.445" y1="0.508" x2="-4.445" y2="-0.508" width="0.1524" layer="21" curve="-180"/>
<wire x1="-4.445" y1="-1.6002" x2="4.445" y2="-1.6002" width="0.0508" layer="21"/>
<smd name="1" x="-3.81" y="-3.0734" dx="0.6604" dy="2.032" layer="1"/>
<smd name="14" x="-3.81" y="3.0734" dx="0.6604" dy="2.032" layer="1"/>
<smd name="2" x="-2.54" y="-3.0734" dx="0.6604" dy="2.032" layer="1"/>
<smd name="3" x="-1.27" y="-3.0734" dx="0.6604" dy="2.032" layer="1"/>
<smd name="13" x="-2.54" y="3.0734" dx="0.6604" dy="2.032" layer="1"/>
<smd name="12" x="-1.27" y="3.0734" dx="0.6604" dy="2.032" layer="1"/>
<smd name="4" x="0" y="-3.0734" dx="0.6604" dy="2.032" layer="1"/>
<smd name="11" x="0" y="3.0734" dx="0.6604" dy="2.032" layer="1"/>
<smd name="5" x="1.27" y="-3.0734" dx="0.6604" dy="2.032" layer="1"/>
<smd name="6" x="2.54" y="-3.0734" dx="0.6604" dy="2.032" layer="1"/>
<smd name="10" x="1.27" y="3.0734" dx="0.6604" dy="2.032" layer="1"/>
<smd name="9" x="2.54" y="3.0734" dx="0.6604" dy="2.032" layer="1"/>
<smd name="7" x="3.81" y="-3.0734" dx="0.6604" dy="2.032" layer="1"/>
<smd name="8" x="3.81" y="3.0734" dx="0.6604" dy="2.032" layer="1"/>
<text x="-3.175" y="-0.762" size="1.27" layer="27" ratio="10">&gt;VALUE</text>
<text x="-4.826" y="-1.905" size="1.27" layer="25" ratio="10" rot="R90">&gt;NAME</text>
<rectangle x1="-0.254" y1="1.9558" x2="0.254" y2="3.0988" layer="51"/>
<rectangle x1="-4.064" y1="-3.0988" x2="-3.556" y2="-1.9558" layer="51"/>
<rectangle x1="-2.794" y1="-3.0988" x2="-2.286" y2="-1.9558" layer="51"/>
<rectangle x1="-1.524" y1="-3.0734" x2="-1.016" y2="-1.9304" layer="51"/>
<rectangle x1="-0.254" y1="-3.0988" x2="0.254" y2="-1.9558" layer="51"/>
<rectangle x1="-1.524" y1="1.9558" x2="-1.016" y2="3.0988" layer="51"/>
<rectangle x1="-2.794" y1="1.9558" x2="-2.286" y2="3.0988" layer="51"/>
<rectangle x1="-4.064" y1="1.9558" x2="-3.556" y2="3.0988" layer="51"/>
<rectangle x1="1.016" y1="1.9558" x2="1.524" y2="3.0988" layer="51"/>
<rectangle x1="2.286" y1="1.9558" x2="2.794" y2="3.0988" layer="51"/>
<rectangle x1="3.556" y1="1.9558" x2="4.064" y2="3.0988" layer="51"/>
<rectangle x1="1.016" y1="-3.0988" x2="1.524" y2="-1.9558" layer="51"/>
<rectangle x1="2.286" y1="-3.0988" x2="2.794" y2="-1.9558" layer="51"/>
<rectangle x1="3.556" y1="-3.0988" x2="4.064" y2="-1.9558" layer="51"/>
</package>
<package name="LCC20" urn="urn:adsk.eagle:footprint:1641/1" library_version="1">
<description>&lt;b&gt;Leadless Chip Carrier&lt;/b&gt;&lt;p&gt; Ceramic Package</description>
<wire x1="-0.4001" y1="4.4" x2="-0.87" y2="4.4" width="0.2032" layer="51"/>
<wire x1="-3.3" y1="4.4" x2="-4.4" y2="3.3" width="0.2032" layer="51"/>
<wire x1="-0.4001" y1="4.3985" x2="0.4001" y2="4.3985" width="0.2032" layer="51" curve="180"/>
<wire x1="-1.6701" y1="4.3985" x2="-0.8699" y2="4.3985" width="0.2032" layer="51" curve="180"/>
<wire x1="-4.3985" y1="2.14" x2="-4.3985" y2="2.94" width="0.2032" layer="51" curve="180"/>
<wire x1="-2.9401" y1="4.4" x2="-3.3" y2="4.4" width="0.2032" layer="51"/>
<wire x1="0.87" y1="4.4" x2="0.4001" y2="4.4" width="0.2032" layer="51"/>
<wire x1="0.87" y1="4.3985" x2="1.67" y2="4.3985" width="0.2032" layer="51" curve="180"/>
<wire x1="-4.4" y1="3.3" x2="-4.4" y2="2.9401" width="0.2032" layer="51"/>
<wire x1="-4.4" y1="2.14" x2="-4.4" y2="1.6701" width="0.2032" layer="51"/>
<wire x1="-4.3985" y1="0.87" x2="-4.3985" y2="1.67" width="0.2032" layer="51" curve="180"/>
<wire x1="-4.3985" y1="-0.4001" x2="-4.3985" y2="0.4001" width="0.2032" layer="51" curve="180"/>
<wire x1="-4.3985" y1="-1.6701" x2="-4.3985" y2="-0.8699" width="0.2032" layer="51" curve="180"/>
<wire x1="-4.4" y1="0.87" x2="-4.4" y2="0.4001" width="0.2032" layer="51"/>
<wire x1="-4.4" y1="-0.4001" x2="-4.4" y2="-0.87" width="0.2032" layer="51"/>
<wire x1="-4.4" y1="-2.9401" x2="-4.4" y2="-4.4" width="0.2032" layer="51"/>
<wire x1="-4.4" y1="-4.4" x2="-4.4" y2="-4.4099" width="0.2032" layer="51"/>
<wire x1="2.14" y1="4.3985" x2="2.94" y2="4.3985" width="0.2032" layer="51" curve="180"/>
<wire x1="2.14" y1="4.4" x2="1.6701" y2="4.4" width="0.2032" layer="51"/>
<wire x1="4.4" y1="4.4" x2="2.9401" y2="4.4" width="0.2032" layer="51"/>
<wire x1="0.4001" y1="-4.4" x2="0.87" y2="-4.4" width="0.2032" layer="51"/>
<wire x1="-0.4001" y1="-4.3985" x2="0.4001" y2="-4.3985" width="0.2032" layer="51" curve="-180"/>
<wire x1="0.87" y1="-4.3985" x2="1.67" y2="-4.3985" width="0.2032" layer="51" curve="-180"/>
<wire x1="2.9401" y1="-4.4" x2="4.4" y2="-4.4" width="0.2032" layer="51"/>
<wire x1="-0.87" y1="-4.4" x2="-0.4001" y2="-4.4" width="0.2032" layer="51"/>
<wire x1="-1.6701" y1="-4.3985" x2="-0.8699" y2="-4.3985" width="0.2032" layer="51" curve="-180"/>
<wire x1="-2.9401" y1="-4.3985" x2="-2.1399" y2="-4.3985" width="0.2032" layer="51" curve="-180"/>
<wire x1="-2.14" y1="-4.4" x2="-1.6701" y2="-4.4" width="0.2032" layer="51"/>
<wire x1="-4.4" y1="-4.4" x2="-2.9401" y2="-4.4" width="0.2032" layer="51"/>
<wire x1="4.4" y1="0.4001" x2="4.4" y2="0.87" width="0.2032" layer="51"/>
<wire x1="4.3985" y1="0.4001" x2="4.3985" y2="-0.4001" width="0.2032" layer="51" curve="180"/>
<wire x1="4.3985" y1="1.6701" x2="4.3985" y2="0.8699" width="0.2032" layer="51" curve="180"/>
<wire x1="4.4" y1="2.9401" x2="4.4" y2="4.4" width="0.2032" layer="51"/>
<wire x1="4.4" y1="-0.87" x2="4.4" y2="-0.4001" width="0.2032" layer="51"/>
<wire x1="4.3985" y1="-0.87" x2="4.3985" y2="-1.67" width="0.2032" layer="51" curve="180"/>
<wire x1="4.3985" y1="-2.14" x2="4.3985" y2="-2.94" width="0.2032" layer="51" curve="180"/>
<wire x1="4.4" y1="-2.14" x2="4.4" y2="-1.6701" width="0.2032" layer="51"/>
<wire x1="4.4" y1="-4.4" x2="4.4" y2="-2.9401" width="0.2032" layer="51"/>
<wire x1="-2.9401" y1="4.3985" x2="-2.1399" y2="4.3985" width="0.2032" layer="51" curve="180"/>
<wire x1="-1.6701" y1="4.4" x2="-2.14" y2="4.4" width="0.2032" layer="51"/>
<wire x1="-4.3985" y1="-2.9401" x2="-4.3985" y2="-2.1399" width="0.2032" layer="51" curve="180"/>
<wire x1="-4.4" y1="-1.6701" x2="-4.4" y2="-2.14" width="0.2032" layer="51"/>
<wire x1="1.6701" y1="-4.4" x2="2.14" y2="-4.4" width="0.2032" layer="51"/>
<wire x1="2.14" y1="-4.3985" x2="2.94" y2="-4.3985" width="0.2032" layer="51" curve="-180"/>
<wire x1="4.3985" y1="2.9401" x2="4.3985" y2="2.1399" width="0.2032" layer="51" curve="180"/>
<wire x1="4.4" y1="1.6701" x2="4.4" y2="2.14" width="0.2032" layer="51"/>
<wire x1="-3.3" y1="4.4" x2="-4.4" y2="3.3" width="0.2032" layer="21"/>
<wire x1="-4.4" y1="-3.1941" x2="-4.4" y2="-4.4" width="0.2032" layer="21"/>
<wire x1="-4.4" y1="-4.4" x2="-3.1941" y2="-4.4" width="0.2032" layer="21"/>
<wire x1="3.1941" y1="-4.4" x2="4.4" y2="-4.4" width="0.2032" layer="21"/>
<wire x1="4.4" y1="-4.4" x2="4.4" y2="-3.1941" width="0.2032" layer="21"/>
<wire x1="4.4" y1="3.1941" x2="4.4" y2="4.4" width="0.2032" layer="21"/>
<wire x1="4.4" y1="4.4" x2="3.1941" y2="4.4" width="0.2032" layer="21"/>
<smd name="2" x="-1.27" y="4.5001" dx="0.8" dy="2" layer="1"/>
<smd name="1" x="0" y="3.8001" dx="0.8" dy="3.4" layer="1"/>
<smd name="3" x="-2.54" y="4.5001" dx="0.8" dy="2" layer="1"/>
<smd name="4" x="-4.5001" y="2.54" dx="2" dy="0.8" layer="1"/>
<smd name="5" x="-4.5001" y="1.27" dx="2" dy="0.8" layer="1"/>
<smd name="6" x="-4.5001" y="0" dx="2" dy="0.8" layer="1"/>
<smd name="7" x="-4.5001" y="-1.27" dx="2" dy="0.8" layer="1"/>
<smd name="8" x="-4.5001" y="-2.54" dx="2" dy="0.8" layer="1"/>
<smd name="9" x="-2.54" y="-4.5001" dx="0.8" dy="2" layer="1"/>
<smd name="10" x="-1.27" y="-4.5001" dx="0.8" dy="2" layer="1"/>
<smd name="11" x="0" y="-4.5001" dx="0.8" dy="2" layer="1"/>
<smd name="12" x="1.27" y="-4.5001" dx="0.8" dy="2" layer="1"/>
<smd name="13" x="2.54" y="-4.5001" dx="0.8" dy="2" layer="1"/>
<smd name="14" x="4.5001" y="-2.54" dx="2" dy="0.8" layer="1"/>
<smd name="15" x="4.5001" y="-1.27" dx="2" dy="0.8" layer="1"/>
<smd name="16" x="4.5001" y="0" dx="2" dy="0.8" layer="1"/>
<smd name="17" x="4.5001" y="1.27" dx="2" dy="0.8" layer="1"/>
<smd name="18" x="4.5001" y="2.54" dx="2" dy="0.8" layer="1"/>
<smd name="19" x="2.54" y="4.5001" dx="0.8" dy="2" layer="1"/>
<smd name="20" x="1.27" y="4.5001" dx="0.8" dy="2" layer="1"/>
<text x="-4.0051" y="6.065" size="1.778" layer="25">&gt;NAME</text>
<text x="-3.9751" y="-7.5601" size="1.778" layer="27">&gt;VALUE</text>
</package>
</packages>
<packages3d>
<package3d name="DIL14" urn="urn:adsk.eagle:package:2008/1" type="box" library_version="1">
<description>Dual In Line Package</description>
</package3d>
<package3d name="SO14" urn="urn:adsk.eagle:package:2007/1" type="box" library_version="1">
<description>Small Outline package 150 mil</description>
</package3d>
<package3d name="LCC20" urn="urn:adsk.eagle:package:2012/1" type="box" library_version="1">
<description>Leadless Chip Carrier Ceramic Package</description>
</package3d>
</packages3d>
<symbols>
<symbol name="7400" urn="urn:adsk.eagle:symbol:1806/1" library_version="1">
<wire x1="-2.54" y1="5.08" x2="-2.54" y2="-5.08" width="0.4064" layer="94"/>
<wire x1="-2.54" y1="5.08" x2="-2.54" y2="-5.08" width="0.4064" layer="94" curve="-180"/>
<text x="2.54" y="3.175" size="1.778" layer="95">&gt;NAME</text>
<text x="2.54" y="-5.08" size="1.778" layer="96">&gt;VALUE</text>
<pin name="I0" x="-7.62" y="2.54" visible="pad" length="middle" direction="in" swaplevel="1"/>
<pin name="I1" x="-7.62" y="-2.54" visible="pad" length="middle" direction="in" swaplevel="1"/>
<pin name="O" x="7.62" y="0" visible="pad" length="middle" direction="out" function="dot" rot="R180"/>
</symbol>
<symbol name="PWRN" urn="urn:adsk.eagle:symbol:1632/1" library_version="1">
<text x="-0.635" y="-0.635" size="1.778" layer="95">&gt;NAME</text>
<text x="1.905" y="-5.842" size="1.27" layer="95" rot="R90">GND</text>
<text x="1.905" y="2.54" size="1.27" layer="95" rot="R90">VCC</text>
<pin name="GND" x="0" y="-7.62" visible="pad" length="middle" direction="pwr" rot="R90"/>
<pin name="VCC" x="0" y="7.62" visible="pad" length="middle" direction="pwr" rot="R270"/>
</symbol>
</symbols>
<devicesets>
<deviceset name="74*00" urn="urn:adsk.eagle:component:2196/1" prefix="IC" library_version="1">
<description>Quad 2-input &lt;b&gt;NAND&lt;/b&gt; gate</description>
<gates>
<gate name="A" symbol="7400" x="20.32" y="0" swaplevel="1"/>
<gate name="B" symbol="7400" x="20.32" y="-12.7" swaplevel="1"/>
<gate name="C" symbol="7400" x="43.18" y="0" swaplevel="1"/>
<gate name="D" symbol="7400" x="43.18" y="-12.7" swaplevel="1"/>
<gate name="P" symbol="PWRN" x="5.08" y="-5.08" addlevel="request"/>
</gates>
<devices>
<device name="N" package="DIL14">
<connects>
<connect gate="A" pin="I0" pad="1"/>
<connect gate="A" pin="I1" pad="2"/>
<connect gate="A" pin="O" pad="3"/>
<connect gate="B" pin="I0" pad="4"/>
<connect gate="B" pin="I1" pad="5"/>
<connect gate="B" pin="O" pad="6"/>
<connect gate="C" pin="I0" pad="9"/>
<connect gate="C" pin="I1" pad="10"/>
<connect gate="C" pin="O" pad="8"/>
<connect gate="D" pin="I0" pad="12"/>
<connect gate="D" pin="I1" pad="13"/>
<connect gate="D" pin="O" pad="11"/>
<connect gate="P" pin="GND" pad="7"/>
<connect gate="P" pin="VCC" pad="14"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:2008/1"/>
</package3dinstances>
<technologies>
<technology name=""/>
<technology name="AC"/>
<technology name="ACT"/>
<technology name="ALS"/>
<technology name="AS"/>
<technology name="HC"/>
<technology name="HCT"/>
<technology name="LS"/>
<technology name="S"/>
</technologies>
</device>
<device name="D" package="SO14">
<connects>
<connect gate="A" pin="I0" pad="1"/>
<connect gate="A" pin="I1" pad="2"/>
<connect gate="A" pin="O" pad="3"/>
<connect gate="B" pin="I0" pad="4"/>
<connect gate="B" pin="I1" pad="5"/>
<connect gate="B" pin="O" pad="6"/>
<connect gate="C" pin="I0" pad="9"/>
<connect gate="C" pin="I1" pad="10"/>
<connect gate="C" pin="O" pad="8"/>
<connect gate="D" pin="I0" pad="12"/>
<connect gate="D" pin="I1" pad="13"/>
<connect gate="D" pin="O" pad="11"/>
<connect gate="P" pin="GND" pad="7"/>
<connect gate="P" pin="VCC" pad="14"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:2007/1"/>
</package3dinstances>
<technologies>
<technology name=""/>
<technology name="AC"/>
<technology name="ACT"/>
<technology name="ALS"/>
<technology name="AS"/>
<technology name="HC"/>
<technology name="HCT"/>
<technology name="LS"/>
<technology name="S"/>
</technologies>
</device>
<device name="FK" package="LCC20">
<connects>
<connect gate="A" pin="I0" pad="2"/>
<connect gate="A" pin="I1" pad="3"/>
<connect gate="A" pin="O" pad="4"/>
<connect gate="B" pin="I0" pad="6"/>
<connect gate="B" pin="I1" pad="8"/>
<connect gate="B" pin="O" pad="9"/>
<connect gate="C" pin="I0" pad="13"/>
<connect gate="C" pin="I1" pad="14"/>
<connect gate="C" pin="O" pad="12"/>
<connect gate="D" pin="I0" pad="18"/>
<connect gate="D" pin="I1" pad="19"/>
<connect gate="D" pin="O" pad="16"/>
<connect gate="P" pin="GND" pad="10"/>
<connect gate="P" pin="VCC" pad="20"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:2012/1"/>
</package3dinstances>
<technologies>
<technology name=""/>
<technology name="AC"/>
<technology name="ACT"/>
<technology name="ALS"/>
<technology name="AS"/>
<technology name="HC"/>
<technology name="HCT"/>
<technology name="LS"/>
<technology name="S"/>
</technologies>
</device>
</devices>
</deviceset>
</devicesets>
</library>
<library name="relay" urn="urn:adsk.eagle:library:339">
<description>&lt;b&gt;Relays&lt;/b&gt;&lt;p&gt;
&lt;ul&gt;
&lt;li&gt;Eichhoff
&lt;li&gt;Finder
&lt;li&gt;Fujitsu
&lt;li&gt;HAMLIN
&lt;li&gt;OMRON
&lt;li&gt;Matsushita
&lt;li&gt;NAiS
&lt;li&gt;Siemens
&lt;li&gt;Schrack
&lt;/ul&gt;
&lt;author&gt;Created by librarian@cadsoft.de&lt;/author&gt;</description>
<packages>
<package name="CQ1" urn="urn:adsk.eagle:footprint:24139/1" library_version="1">
<description>&lt;b&gt;AUTOMOTIVE QUIET RELAY&lt;/b&gt; NAiS&lt;p&gt;
Source: http://www.mew-europe.com/.. en_ds_61208_0000.pdf</description>
<wire x1="-7.8" y1="6.4" x2="7.8" y2="6.4" width="0.2032" layer="21"/>
<wire x1="8.4" y1="5.8" x2="8.4" y2="-5.8" width="0.2032" layer="21"/>
<wire x1="7.8" y1="-6.4" x2="-7.8" y2="-6.4" width="0.2032" layer="21"/>
<wire x1="-8.4" y1="-5.8" x2="-8.4" y2="5.8" width="0.2032" layer="21"/>
<wire x1="-8.4" y1="5.8" x2="-7.8" y2="6.4" width="0.2032" layer="21" curve="-90"/>
<wire x1="7.8" y1="6.4" x2="8.4" y2="5.8" width="0.2032" layer="21" curve="-90"/>
<wire x1="8.4" y1="-5.8" x2="7.8" y2="-6.4" width="0.2032" layer="21" curve="-90"/>
<wire x1="-7.8" y1="-6.4" x2="-8.4" y2="-5.8" width="0.2032" layer="21" curve="-90"/>
<pad name="NO" x="-5.7" y="5" drill="1.5" rot="R180"/>
<pad name="NC" x="-5.7" y="-5" drill="1.5" rot="R180"/>
<pad name="1" x="4.5" y="-5" drill="1.5" rot="R180"/>
<pad name="2" x="4.5" y="5" drill="1.5" rot="R180"/>
<pad name="C" x="7" y="0" drill="1.5" rot="R180"/>
<text x="-7.62" y="1.27" size="1.27" layer="25">&gt;NAME</text>
<text x="-7.62" y="-1.27" size="1.27" layer="27">&gt;VALUE</text>
</package>
</packages>
<packages3d>
<package3d name="CQ1" urn="urn:adsk.eagle:package:24445/1" type="box" library_version="1">
<description>AUTOMOTIVE QUIET RELAY NAiS
Source: http://www.mew-europe.com/.. en_ds_61208_0000.pdf</description>
</package3d>
</packages3d>
<symbols>
<symbol name="K" urn="urn:adsk.eagle:symbol:23941/1" library_version="1">
<wire x1="-3.81" y1="-1.905" x2="-1.905" y2="-1.905" width="0.254" layer="94"/>
<wire x1="3.81" y1="-1.905" x2="3.81" y2="1.905" width="0.254" layer="94"/>
<wire x1="3.81" y1="1.905" x2="1.905" y2="1.905" width="0.254" layer="94"/>
<wire x1="-3.81" y1="1.905" x2="-3.81" y2="-1.905" width="0.254" layer="94"/>
<wire x1="0" y1="-2.54" x2="0" y2="-1.905" width="0.1524" layer="94"/>
<wire x1="0" y1="-1.905" x2="3.81" y2="-1.905" width="0.254" layer="94"/>
<wire x1="0" y1="2.54" x2="0" y2="1.905" width="0.1524" layer="94"/>
<wire x1="0" y1="1.905" x2="-3.81" y2="1.905" width="0.254" layer="94"/>
<wire x1="-1.905" y1="-1.905" x2="1.905" y2="1.905" width="0.1524" layer="94"/>
<wire x1="-1.905" y1="-1.905" x2="0" y2="-1.905" width="0.254" layer="94"/>
<wire x1="1.905" y1="1.905" x2="0" y2="1.905" width="0.254" layer="94"/>
<text x="1.27" y="2.921" size="1.778" layer="96">&gt;VALUE</text>
<text x="1.27" y="5.08" size="1.778" layer="95">&gt;PART</text>
<pin name="2" x="0" y="-5.08" visible="pad" length="short" direction="pas" rot="R90"/>
<pin name="1" x="0" y="5.08" visible="pad" length="short" direction="pas" rot="R270"/>
</symbol>
<symbol name="U" urn="urn:adsk.eagle:symbol:23944/1" library_version="1">
<wire x1="3.175" y1="5.08" x2="1.905" y2="5.08" width="0.254" layer="94"/>
<wire x1="-3.175" y1="5.08" x2="-1.905" y2="5.08" width="0.254" layer="94"/>
<wire x1="0" y1="1.27" x2="2.54" y2="5.715" width="0.254" layer="94"/>
<wire x1="0" y1="1.27" x2="0" y2="0" width="0.254" layer="94"/>
<circle x="0" y="1.27" radius="0.127" width="0.4064" layer="94"/>
<text x="2.54" y="0" size="1.778" layer="95">&gt;PART</text>
<pin name="O" x="5.08" y="5.08" visible="pad" length="short" direction="pas" rot="R180"/>
<pin name="S" x="-5.08" y="5.08" visible="pad" length="short" direction="pas"/>
<pin name="P" x="0" y="-2.54" visible="pad" length="short" direction="pas" rot="R90"/>
</symbol>
</symbols>
<devicesets>
<deviceset name="CQ1-12V" urn="urn:adsk.eagle:component:24708/1" prefix="K" library_version="1">
<description>&lt;b&gt;AUTOMOTIVE QUIET RELAY&lt;/b&gt; NAiS&lt;p&gt;
Source: http://www.mew-europe.com/.. en_ds_61208_0000.pdf</description>
<gates>
<gate name="1" symbol="K" x="-7.62" y="0" addlevel="must"/>
<gate name="2" symbol="U" x="10.16" y="0" addlevel="always"/>
</gates>
<devices>
<device name="" package="CQ1">
<connects>
<connect gate="1" pin="1" pad="1"/>
<connect gate="1" pin="2" pad="2"/>
<connect gate="2" pin="O" pad="NC"/>
<connect gate="2" pin="P" pad="C"/>
<connect gate="2" pin="S" pad="NO"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:24445/1"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="MF" value="" constant="no"/>
<attribute name="MPN" value="" constant="no"/>
<attribute name="OC_FARNELL" value="unknown" constant="no"/>
<attribute name="OC_NEWARK" value="unknown" constant="no"/>
</technology>
</technologies>
</device>
</devices>
</deviceset>
</devicesets>
</library>
<library name="con-wago-500" urn="urn:adsk.eagle:library:195">
<description>&lt;b&gt;Wago Screw Clamps&lt;/b&gt;&lt;p&gt;
Grid 5.00 mm&lt;p&gt;
&lt;author&gt;Created by librarian@cadsoft.de&lt;/author&gt;</description>
<packages>
<package name="W237-102" urn="urn:adsk.eagle:footprint:10676/1" library_version="1">
<description>&lt;b&gt;WAGO SCREW CLAMP&lt;/b&gt;</description>
<wire x1="-3.491" y1="-2.286" x2="-1.484" y2="-0.279" width="0.254" layer="51"/>
<wire x1="1.488" y1="-2.261" x2="3.469" y2="-0.254" width="0.254" layer="51"/>
<wire x1="-4.989" y1="-5.461" x2="4.993" y2="-5.461" width="0.1524" layer="21"/>
<wire x1="4.993" y1="3.734" x2="4.993" y2="3.531" width="0.1524" layer="21"/>
<wire x1="4.993" y1="3.734" x2="-4.989" y2="3.734" width="0.1524" layer="21"/>
<wire x1="-4.989" y1="-5.461" x2="-4.989" y2="-3.073" width="0.1524" layer="21"/>
<wire x1="-4.989" y1="-3.073" x2="-3.389" y2="-3.073" width="0.1524" layer="21"/>
<wire x1="-3.389" y1="-3.073" x2="-1.611" y2="-3.073" width="0.1524" layer="51"/>
<wire x1="-1.611" y1="-3.073" x2="1.615" y2="-3.073" width="0.1524" layer="21"/>
<wire x1="3.393" y1="-3.073" x2="4.993" y2="-3.073" width="0.1524" layer="21"/>
<wire x1="-4.989" y1="-3.073" x2="-4.989" y2="3.531" width="0.1524" layer="21"/>
<wire x1="4.993" y1="-3.073" x2="4.993" y2="-5.461" width="0.1524" layer="21"/>
<wire x1="-4.989" y1="3.531" x2="4.993" y2="3.531" width="0.1524" layer="21"/>
<wire x1="-4.989" y1="3.531" x2="-4.989" y2="3.734" width="0.1524" layer="21"/>
<wire x1="4.993" y1="3.531" x2="4.993" y2="-3.073" width="0.1524" layer="21"/>
<wire x1="1.615" y1="-3.073" x2="3.393" y2="-3.073" width="0.1524" layer="51"/>
<circle x="-2.5" y="-1.27" radius="1.4986" width="0.1524" layer="51"/>
<circle x="-2.5" y="2.2098" radius="0.508" width="0.1524" layer="21"/>
<circle x="2.5038" y="-1.27" radius="1.4986" width="0.1524" layer="51"/>
<circle x="2.5038" y="2.2098" radius="0.508" width="0.1524" layer="21"/>
<pad name="1" x="-2.5" y="-1.27" drill="1.1938" shape="long" rot="R90"/>
<pad name="2" x="2.5" y="-1.27" drill="1.1938" shape="long" rot="R90"/>
<text x="-5.04" y="-7.62" size="1.27" layer="27" ratio="10">&gt;VALUE</text>
<text x="-3.8462" y="-5.0038" size="1.27" layer="25" ratio="10">&gt;NAME</text>
<text x="-4.532" y="0.635" size="1.27" layer="21" ratio="10">1</text>
<text x="0.421" y="0.635" size="1.27" layer="21" ratio="10">2</text>
</package>
</packages>
<packages3d>
<package3d name="W237-102" urn="urn:adsk.eagle:package:10688/1" type="box" library_version="1">
<description>WAGO SCREW CLAMP</description>
</package3d>
</packages3d>
<symbols>
<symbol name="KL" urn="urn:adsk.eagle:symbol:10675/1" library_version="1">
<circle x="1.27" y="0" radius="1.27" width="0.254" layer="94"/>
<text x="0" y="0.889" size="1.778" layer="95" rot="R180">&gt;NAME</text>
<pin name="KL" x="5.08" y="0" visible="off" length="short" direction="pas" rot="R180"/>
</symbol>
<symbol name="KL+V" urn="urn:adsk.eagle:symbol:10673/1" library_version="1">
<circle x="1.27" y="0" radius="1.27" width="0.254" layer="94"/>
<text x="-2.54" y="-3.683" size="1.778" layer="96">&gt;VALUE</text>
<text x="0" y="0.889" size="1.778" layer="95" rot="R180">&gt;NAME</text>
<pin name="KL" x="5.08" y="0" visible="off" length="short" direction="pas" rot="R180"/>
</symbol>
</symbols>
<devicesets>
<deviceset name="W237-102" urn="urn:adsk.eagle:component:10702/1" prefix="X" uservalue="yes" library_version="1">
<description>&lt;b&gt;WAGO SCREW CLAMP&lt;/b&gt;</description>
<gates>
<gate name="-1" symbol="KL" x="0" y="5.08" addlevel="always"/>
<gate name="-2" symbol="KL+V" x="0" y="0" addlevel="always"/>
</gates>
<devices>
<device name="" package="W237-102">
<connects>
<connect gate="-1" pin="KL" pad="1"/>
<connect gate="-2" pin="KL" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:10688/1"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="MF" value="" constant="no"/>
<attribute name="MPN" value="237-102" constant="no"/>
<attribute name="OC_FARNELL" value="unknown" constant="no"/>
<attribute name="OC_NEWARK" value="70K9898" constant="no"/>
</technology>
</technologies>
</device>
</devices>
</deviceset>
</devicesets>
</library>
<library name="fuse" urn="urn:adsk.eagle:library:233">
<description>&lt;b&gt;Fuses and Fuse Holders&lt;/b&gt;&lt;p&gt;
&lt;author&gt;Created by librarian@cadsoft.de&lt;/author&gt;</description>
<packages>
<package name="GSH15" urn="urn:adsk.eagle:footprint:14041/1" library_version="1">
<description>&lt;b&gt;FUSE HOLDER&lt;/b&gt;&lt;p&gt;
grid 15mm, 19649 Wickmann</description>
<wire x1="-12.573" y1="-4.572" x2="-12.573" y2="4.572" width="0.1524" layer="21"/>
<wire x1="12.573" y1="4.572" x2="12.573" y2="-4.572" width="0.1524" layer="21"/>
<wire x1="-5.08" y1="2.921" x2="5.08" y2="2.921" width="0.0508" layer="21"/>
<wire x1="-11.43" y1="3.683" x2="-8.382" y2="3.683" width="0.1524" layer="21"/>
<wire x1="-11.43" y1="-3.683" x2="-11.43" y2="3.683" width="0.1524" layer="21"/>
<wire x1="11.43" y1="3.683" x2="11.43" y2="-3.683" width="0.1524" layer="21"/>
<wire x1="-5.207" y1="0" x2="5.207" y2="0" width="0.0508" layer="21"/>
<wire x1="12.573" y1="-4.572" x2="5.08" y2="-4.572" width="0.1524" layer="21"/>
<wire x1="5.08" y1="-3.048" x2="5.08" y2="-3.683" width="0.1524" layer="21"/>
<wire x1="5.08" y1="-3.048" x2="-5.08" y2="-3.048" width="0.1524" layer="21"/>
<wire x1="-5.08" y1="-4.572" x2="-12.573" y2="-4.572" width="0.1524" layer="21"/>
<wire x1="-12.573" y1="4.572" x2="-5.08" y2="4.572" width="0.1524" layer="21"/>
<wire x1="-5.08" y1="3.048" x2="5.08" y2="3.048" width="0.1524" layer="21"/>
<wire x1="5.08" y1="4.572" x2="5.08" y2="3.683" width="0.1524" layer="21"/>
<wire x1="5.08" y1="4.572" x2="12.573" y2="4.572" width="0.1524" layer="21"/>
<wire x1="5.08" y1="-3.683" x2="6.858" y2="-3.683" width="0.1524" layer="21"/>
<wire x1="6.858" y1="-4.191" x2="6.858" y2="-3.683" width="0.1524" layer="21"/>
<wire x1="6.858" y1="-4.191" x2="8.382" y2="-4.191" width="0.1524" layer="21"/>
<wire x1="8.382" y1="-3.683" x2="8.382" y2="-4.191" width="0.1524" layer="21"/>
<wire x1="8.382" y1="-3.683" x2="11.43" y2="-3.683" width="0.1524" layer="21"/>
<wire x1="5.08" y1="3.683" x2="6.858" y2="3.683" width="0.1524" layer="21"/>
<wire x1="6.858" y1="4.191" x2="6.858" y2="3.683" width="0.1524" layer="21"/>
<wire x1="6.858" y1="4.191" x2="8.382" y2="4.191" width="0.1524" layer="21"/>
<wire x1="8.382" y1="3.683" x2="8.382" y2="4.191" width="0.1524" layer="21"/>
<wire x1="8.382" y1="3.683" x2="11.43" y2="3.683" width="0.1524" layer="21"/>
<wire x1="5.08" y1="3.683" x2="5.08" y2="3.048" width="0.1524" layer="21"/>
<wire x1="5.08" y1="-3.683" x2="5.08" y2="-4.572" width="0.1524" layer="21"/>
<wire x1="-5.08" y1="3.048" x2="-5.08" y2="3.683" width="0.1524" layer="21"/>
<wire x1="-5.08" y1="3.683" x2="-5.08" y2="4.572" width="0.1524" layer="21"/>
<wire x1="-5.08" y1="-4.572" x2="-5.08" y2="-3.683" width="0.1524" layer="21"/>
<wire x1="-5.08" y1="-3.683" x2="-5.08" y2="-3.048" width="0.1524" layer="21"/>
<wire x1="-6.858" y1="3.683" x2="-5.08" y2="3.683" width="0.1524" layer="21"/>
<wire x1="-6.858" y1="4.191" x2="-6.858" y2="3.683" width="0.1524" layer="21"/>
<wire x1="-8.382" y1="4.191" x2="-6.858" y2="4.191" width="0.1524" layer="21"/>
<wire x1="-8.382" y1="3.683" x2="-8.382" y2="4.191" width="0.1524" layer="21"/>
<wire x1="-6.858" y1="-3.683" x2="-5.08" y2="-3.683" width="0.1524" layer="21"/>
<wire x1="-6.858" y1="-4.191" x2="-6.858" y2="-3.683" width="0.1524" layer="21"/>
<wire x1="-8.382" y1="-4.191" x2="-6.858" y2="-4.191" width="0.1524" layer="21"/>
<wire x1="-8.382" y1="-3.683" x2="-8.382" y2="-4.191" width="0.1524" layer="21"/>
<wire x1="-8.382" y1="-3.683" x2="-11.43" y2="-3.683" width="0.1524" layer="21"/>
<pad name="1" x="-7.62" y="0" drill="1.3208" diameter="2.54" shape="long" rot="R90"/>
<pad name="2" x="7.62" y="0" drill="1.3208" diameter="2.54" shape="long" rot="R90"/>
<text x="-4.4196" y="3.6068" size="1.778" layer="25" ratio="10">&gt;NAME</text>
<text x="-4.4196" y="-5.461" size="1.778" layer="27" ratio="10">&gt;VALUE</text>
<rectangle x1="-5.08" y1="1.651" x2="5.08" y2="2.159" layer="21"/>
<rectangle x1="-5.08" y1="-2.159" x2="5.08" y2="-1.651" layer="21"/>
<rectangle x1="5.08" y1="-3.175" x2="5.715" y2="-1.524" layer="21"/>
<rectangle x1="5.08" y1="1.524" x2="5.715" y2="3.175" layer="21"/>
<rectangle x1="5.08" y1="-1.524" x2="5.715" y2="1.524" layer="51"/>
<rectangle x1="10.287" y1="-2.032" x2="10.541" y2="2.032" layer="21"/>
<rectangle x1="6.731" y1="1.905" x2="8.509" y2="2.032" layer="51"/>
<rectangle x1="6.731" y1="-2.032" x2="8.509" y2="-1.905" layer="51"/>
<rectangle x1="5.715" y1="2.032" x2="10.541" y2="3.175" layer="21"/>
<rectangle x1="5.715" y1="-3.175" x2="10.541" y2="-2.032" layer="21"/>
<rectangle x1="7.112" y1="-3.937" x2="8.128" y2="-3.175" layer="21"/>
<rectangle x1="7.112" y1="3.175" x2="8.128" y2="3.937" layer="21"/>
<rectangle x1="-5.715" y1="-1.524" x2="-5.08" y2="1.524" layer="51"/>
<rectangle x1="-5.715" y1="1.524" x2="-5.08" y2="3.175" layer="21"/>
<rectangle x1="-5.715" y1="-3.175" x2="-5.08" y2="-1.524" layer="21"/>
<rectangle x1="-10.541" y1="-3.175" x2="-5.715" y2="-2.032" layer="21"/>
<rectangle x1="-8.509" y1="-2.032" x2="-6.731" y2="-1.905" layer="51"/>
<rectangle x1="-10.541" y1="-2.032" x2="-10.287" y2="2.032" layer="21"/>
<rectangle x1="-8.509" y1="1.905" x2="-6.731" y2="2.032" layer="51"/>
<rectangle x1="-10.541" y1="2.032" x2="-5.715" y2="3.175" layer="21"/>
<rectangle x1="-8.128" y1="3.175" x2="-7.112" y2="3.937" layer="21"/>
<rectangle x1="-8.128" y1="-3.937" x2="-7.112" y2="-3.175" layer="21"/>
<rectangle x1="5.715" y1="-2.032" x2="6.731" y2="-1.905" layer="21"/>
<rectangle x1="5.715" y1="1.905" x2="6.731" y2="2.032" layer="21"/>
<rectangle x1="8.509" y1="1.905" x2="10.287" y2="2.032" layer="21"/>
<rectangle x1="8.509" y1="-2.032" x2="10.287" y2="-1.905" layer="21"/>
<rectangle x1="-10.287" y1="1.905" x2="-8.509" y2="2.032" layer="21"/>
<rectangle x1="-6.731" y1="1.905" x2="-5.715" y2="2.032" layer="21"/>
<rectangle x1="-6.731" y1="-2.032" x2="-5.715" y2="-1.905" layer="21"/>
<rectangle x1="-10.287" y1="-2.032" x2="-8.509" y2="-1.905" layer="21"/>
<hole x="0" y="0" drill="2.794"/>
</package>
</packages>
<packages3d>
<package3d name="GSH15" urn="urn:adsk.eagle:package:14065/1" type="box" library_version="1">
<description>FUSE HOLDER
grid 15mm, 19649 Wickmann</description>
</package3d>
</packages3d>
<symbols>
<symbol name="FUSE" urn="urn:adsk.eagle:symbol:14027/1" library_version="1">
<wire x1="-3.81" y1="-0.762" x2="3.81" y2="-0.762" width="0.254" layer="94"/>
<wire x1="3.81" y1="0.762" x2="-3.81" y2="0.762" width="0.254" layer="94"/>
<wire x1="3.81" y1="-0.762" x2="3.81" y2="0.762" width="0.254" layer="94"/>
<wire x1="-3.81" y1="0.762" x2="-3.81" y2="-0.762" width="0.254" layer="94"/>
<wire x1="-2.54" y1="0" x2="2.54" y2="0" width="0.1524" layer="94"/>
<text x="-3.81" y="1.397" size="1.778" layer="95">&gt;NAME</text>
<text x="-3.81" y="-2.921" size="1.778" layer="96">&gt;VALUE</text>
<pin name="2" x="5.08" y="0" visible="off" length="short" direction="pas" swaplevel="1" rot="R180"/>
<pin name="1" x="-5.08" y="0" visible="off" length="short" direction="pas" swaplevel="1"/>
</symbol>
</symbols>
<devicesets>
<deviceset name="GSH15" urn="urn:adsk.eagle:component:14090/1" prefix="F" uservalue="yes" library_version="1">
<description>&lt;b&gt;FUSE HOLDER&lt;/b&gt;&lt;p&gt;
grid 15mm, 19649 Wickmann</description>
<gates>
<gate name="1" symbol="FUSE" x="0" y="0"/>
</gates>
<devices>
<device name="" package="GSH15">
<connects>
<connect gate="1" pin="1" pad="1"/>
<connect gate="1" pin="2" pad="2"/>
</connects>
<package3dinstances>
<package3dinstance package3d_urn="urn:adsk.eagle:package:14065/1"/>
</package3dinstances>
<technologies>
<technology name="">
<attribute name="MF" value="" constant="no"/>
<attribute name="MPN" value="64900001039" constant="no"/>
<attribute name="OC_FARNELL" value="1271673" constant="no"/>
<attribute name="OC_NEWARK" value="02P0318" constant="no"/>
</technology>
</technologies>
</device>
</devices>
</deviceset>
</devicesets>
</library>
</libraries>
<attributes>
</attributes>
<variantdefs>
</variantdefs>
<classes>
<class number="0" name="default" width="0" drill="0">
</class>
</classes>
<parts>
<part name="J1" library="Wurth_Elektronik_Electromechanic_Wire_to_Board_Connectors_rev17b" library_urn="urn:adsk.eagle:library:493" deviceset="61202421621" device=""/>
<part name="LEFT" library="Wurth_Elektronik_Electromechanic_Wire_to_Board_Connectors_rev17b" library_urn="urn:adsk.eagle:library:493" deviceset="61201421621" device=""/>
<part name="BACK" library="Wurth_Elektronik_Electromechanic_Wire_to_Board_Connectors_rev17b" library_urn="urn:adsk.eagle:library:493" deviceset="61201421621" device=""/>
<part name="RIGHT" library="Wurth_Elektronik_Electromechanic_Wire_to_Board_Connectors_rev17b" library_urn="urn:adsk.eagle:library:493" deviceset="61201421621" device=""/>
<part name="FRONT" library="Wurth_Elektronik_Electromechanic_Wire_to_Board_Connectors_rev17b" library_urn="urn:adsk.eagle:library:493" deviceset="61201421621" device=""/>
<part name="ARM" library="Wurth_Elektronik_Electromechanic_Wire_to_Board_Connectors_rev17b" library_urn="urn:adsk.eagle:library:493" deviceset="61201421621" device=""/>
<part name="EXTRA" library="Wurth_Elektronik_Electromechanic_Wire_to_Board_Connectors_rev17b" library_urn="urn:adsk.eagle:library:493" deviceset="61201421621" device=""/>
<part name="IC2" library="uln-udn" library_urn="urn:adsk.eagle:library:407" deviceset="ULN2003A" device="N" package3d_urn="urn:adsk.eagle:package:30239/1"/>
<part name="IC1" library="74xx-eu" library_urn="urn:adsk.eagle:library:85" deviceset="74*00" device="N" package3d_urn="urn:adsk.eagle:package:2008/1" technology="AC"/>
<part name="ARMBRAKE" library="relay" library_urn="urn:adsk.eagle:library:339" deviceset="CQ1-12V" device="" package3d_urn="urn:adsk.eagle:package:24445/1"/>
<part name="BRAKEF" library="relay" library_urn="urn:adsk.eagle:library:339" deviceset="CQ1-12V" device="" package3d_urn="urn:adsk.eagle:package:24445/1"/>
<part name="BRAKEL" library="relay" library_urn="urn:adsk.eagle:library:339" deviceset="CQ1-12V" device="" package3d_urn="urn:adsk.eagle:package:24445/1"/>
<part name="BRAKEB" library="relay" library_urn="urn:adsk.eagle:library:339" deviceset="CQ1-12V" device="" package3d_urn="urn:adsk.eagle:package:24445/1"/>
<part name="BRAKER" library="relay" library_urn="urn:adsk.eagle:library:339" deviceset="CQ1-12V" device="" package3d_urn="urn:adsk.eagle:package:24445/1"/>
<part name="SUPPLY" library="con-wago-500" library_urn="urn:adsk.eagle:library:195" deviceset="W237-102" device="" package3d_urn="urn:adsk.eagle:package:10688/1"/>
<part name="KILL" library="con-wago-500" library_urn="urn:adsk.eagle:library:195" deviceset="W237-102" device="" package3d_urn="urn:adsk.eagle:package:10688/1"/>
<part name="F1" library="fuse" library_urn="urn:adsk.eagle:library:233" deviceset="GSH15" device="" package3d_urn="urn:adsk.eagle:package:14065/1"/>
<part name="SUPPLY1" library="con-wago-500" library_urn="urn:adsk.eagle:library:195" deviceset="W237-102" device="" package3d_urn="urn:adsk.eagle:package:10688/1"/>
<part name="SUPPLY2" library="con-wago-500" library_urn="urn:adsk.eagle:library:195" deviceset="W237-102" device="" package3d_urn="urn:adsk.eagle:package:10688/1"/>
<part name="SUPPLY3" library="con-wago-500" library_urn="urn:adsk.eagle:library:195" deviceset="W237-102" device="" package3d_urn="urn:adsk.eagle:package:10688/1"/>
<part name="SUPPLY4" library="con-wago-500" library_urn="urn:adsk.eagle:library:195" deviceset="W237-102" device="" package3d_urn="urn:adsk.eagle:package:10688/1"/>
<part name="SUPPLY5" library="con-wago-500" library_urn="urn:adsk.eagle:library:195" deviceset="W237-102" device="" package3d_urn="urn:adsk.eagle:package:10688/1"/>
<part name="SUPPLY6" library="con-wago-500" library_urn="urn:adsk.eagle:library:195" deviceset="W237-102" device="" package3d_urn="urn:adsk.eagle:package:10688/1"/>
<part name="X1" library="con-wago-500" library_urn="urn:adsk.eagle:library:195" deviceset="W237-102" device="" package3d_urn="urn:adsk.eagle:package:10688/1"/>
<part name="X2" library="con-wago-500" library_urn="urn:adsk.eagle:library:195" deviceset="W237-102" device="" package3d_urn="urn:adsk.eagle:package:10688/1"/>
<part name="X3" library="con-wago-500" library_urn="urn:adsk.eagle:library:195" deviceset="W237-102" device="" package3d_urn="urn:adsk.eagle:package:10688/1"/>
<part name="X4" library="con-wago-500" library_urn="urn:adsk.eagle:library:195" deviceset="W237-102" device="" package3d_urn="urn:adsk.eagle:package:10688/1"/>
<part name="X5" library="con-wago-500" library_urn="urn:adsk.eagle:library:195" deviceset="W237-102" device="" package3d_urn="urn:adsk.eagle:package:10688/1"/>
<part name="X6" library="con-wago-500" library_urn="urn:adsk.eagle:library:195" deviceset="W237-102" device="" package3d_urn="urn:adsk.eagle:package:10688/1"/>
<part name="X7" library="con-wago-500" library_urn="urn:adsk.eagle:library:195" deviceset="W237-102" device="" package3d_urn="urn:adsk.eagle:package:10688/1"/>
<part name="X8" library="con-wago-500" library_urn="urn:adsk.eagle:library:195" deviceset="W237-102" device="" package3d_urn="urn:adsk.eagle:package:10688/1"/>
<part name="X9" library="con-wago-500" library_urn="urn:adsk.eagle:library:195" deviceset="W237-102" device="" package3d_urn="urn:adsk.eagle:package:10688/1"/>
<part name="X10" library="con-wago-500" library_urn="urn:adsk.eagle:library:195" deviceset="W237-102" device="" package3d_urn="urn:adsk.eagle:package:10688/1"/>
</parts>
<sheets>
<sheet>
<plain>
</plain>
<instances>
<instance part="J1" gate="G$1" x="35.56" y="71.12" rot="R270"/>
<instance part="LEFT" gate="G$1" x="-22.86" y="-10.16" rot="R90"/>
<instance part="BACK" gate="G$1" x="-25.4" y="33.02" rot="R90"/>
<instance part="RIGHT" gate="G$1" x="-25.4" y="73.66" rot="R90"/>
<instance part="FRONT" gate="G$1" x="-25.4" y="116.84" rot="R90"/>
<instance part="ARM" gate="G$1" x="78.74" y="121.92" rot="R270"/>
<instance part="EXTRA" gate="G$1" x="78.74" y="78.74" rot="R270"/>
<instance part="IC2" gate="A" x="99.06" y="30.48"/>
<instance part="IC1" gate="A" x="68.58" y="45.72"/>
<instance part="IC1" gate="B" x="68.58" y="27.94"/>
<instance part="ARMBRAKE" gate="1" x="134.62" y="66.04"/>
<instance part="ARMBRAKE" gate="2" x="152.4" y="66.04"/>
<instance part="BRAKEF" gate="1" x="134.62" y="48.26"/>
<instance part="BRAKEF" gate="2" x="152.4" y="48.26"/>
<instance part="BRAKEL" gate="1" x="134.62" y="30.48"/>
<instance part="BRAKEL" gate="2" x="152.4" y="30.48"/>
<instance part="BRAKEB" gate="1" x="134.62" y="12.7"/>
<instance part="BRAKEB" gate="2" x="152.4" y="12.7"/>
<instance part="BRAKER" gate="1" x="134.62" y="-5.08"/>
<instance part="BRAKER" gate="2" x="152.4" y="-5.08"/>
<instance part="SUPPLY" gate="-1" x="-111.76" y="165.1"/>
<instance part="SUPPLY" gate="-2" x="-111.76" y="160.02"/>
<instance part="KILL" gate="-1" x="-111.76" y="129.54"/>
<instance part="KILL" gate="-2" x="-111.76" y="134.62"/>
<instance part="F1" gate="1" x="-93.98" y="149.86" rot="R270"/>
<instance part="SUPPLY1" gate="-1" x="-111.76" y="124.46"/>
<instance part="SUPPLY1" gate="-2" x="-111.76" y="119.38"/>
<instance part="SUPPLY2" gate="-1" x="-111.76" y="111.76"/>
<instance part="SUPPLY2" gate="-2" x="-111.76" y="106.68"/>
<instance part="SUPPLY3" gate="-1" x="-111.76" y="99.06"/>
<instance part="SUPPLY3" gate="-2" x="-111.76" y="93.98"/>
<instance part="SUPPLY4" gate="-1" x="-111.76" y="86.36"/>
<instance part="SUPPLY4" gate="-2" x="-111.76" y="81.28"/>
<instance part="SUPPLY5" gate="-1" x="-111.76" y="73.66"/>
<instance part="SUPPLY5" gate="-2" x="-111.76" y="68.58"/>
<instance part="SUPPLY6" gate="-1" x="-111.76" y="60.96"/>
<instance part="SUPPLY6" gate="-2" x="-111.76" y="55.88"/>
<instance part="X1" gate="-1" x="195.58" y="101.6"/>
<instance part="X1" gate="-2" x="195.58" y="96.52"/>
<instance part="X2" gate="-1" x="195.58" y="88.9"/>
<instance part="X2" gate="-2" x="195.58" y="83.82"/>
<instance part="X3" gate="-1" x="195.58" y="73.66"/>
<instance part="X3" gate="-2" x="195.58" y="68.58"/>
<instance part="X4" gate="-1" x="195.58" y="58.42"/>
<instance part="X4" gate="-2" x="195.58" y="53.34"/>
<instance part="X5" gate="-1" x="195.58" y="43.18"/>
<instance part="X5" gate="-2" x="195.58" y="38.1"/>
<instance part="X6" gate="-1" x="195.58" y="27.94"/>
<instance part="X6" gate="-2" x="195.58" y="22.86"/>
<instance part="X7" gate="-1" x="195.58" y="12.7"/>
<instance part="X7" gate="-2" x="195.58" y="7.62"/>
<instance part="X8" gate="-1" x="195.58" y="-5.08"/>
<instance part="X8" gate="-2" x="195.58" y="-10.16"/>
<instance part="X9" gate="-1" x="195.58" y="-22.86"/>
<instance part="X9" gate="-2" x="195.58" y="-27.94"/>
<instance part="X10" gate="-1" x="195.58" y="-40.64"/>
<instance part="X10" gate="-2" x="195.58" y="-45.72"/>
</instances>
<busses>
</busses>
<nets>
<net name="FIN1" class="0">
<segment>
<wire x1="-15.24" y1="132.08" x2="-10.16" y2="132.08" width="0.1524" layer="91"/>
<pinref part="FRONT" gate="G$1" pin="11"/>
</segment>
<segment>
<pinref part="J1" gate="G$1" pin="1"/>
<wire x1="25.4" y1="93.98" x2="20.32" y2="93.98" width="0.1524" layer="91"/>
</segment>
</net>
<net name="FPWM" class="0">
<segment>
<wire x1="-15.24" y1="121.92" x2="-10.16" y2="121.92" width="0.1524" layer="91"/>
<pinref part="FRONT" gate="G$1" pin="7"/>
</segment>
<segment>
<pinref part="J1" gate="G$1" pin="3"/>
<wire x1="25.4" y1="88.9" x2="20.32" y2="88.9" width="0.1524" layer="91"/>
</segment>
</net>
<net name="FIN2" class="0">
<segment>
<wire x1="-15.24" y1="111.76" x2="-10.16" y2="111.76" width="0.1524" layer="91"/>
<pinref part="FRONT" gate="G$1" pin="3"/>
</segment>
<segment>
<pinref part="J1" gate="G$1" pin="5"/>
<wire x1="25.4" y1="83.82" x2="22.86" y2="83.82" width="0.1524" layer="91"/>
</segment>
</net>
<net name="RIN1" class="0">
<segment>
<wire x1="-15.24" y1="88.9" x2="-12.7" y2="88.9" width="0.1524" layer="91"/>
<pinref part="RIGHT" gate="G$1" pin="11"/>
</segment>
<segment>
<pinref part="J1" gate="G$1" pin="7"/>
<wire x1="25.4" y1="78.74" x2="20.32" y2="78.74" width="0.1524" layer="91"/>
</segment>
</net>
<net name="RPWM" class="0">
<segment>
<wire x1="-15.24" y1="78.74" x2="-10.16" y2="78.74" width="0.1524" layer="91"/>
<pinref part="RIGHT" gate="G$1" pin="7"/>
</segment>
<segment>
<pinref part="J1" gate="G$1" pin="9"/>
<wire x1="25.4" y1="73.66" x2="20.32" y2="73.66" width="0.1524" layer="91"/>
</segment>
</net>
<net name="RIN2" class="0">
<segment>
<wire x1="-15.24" y1="68.58" x2="-10.16" y2="68.58" width="0.1524" layer="91"/>
<pinref part="RIGHT" gate="G$1" pin="3"/>
</segment>
<segment>
<pinref part="J1" gate="G$1" pin="11"/>
<wire x1="25.4" y1="68.58" x2="20.32" y2="68.58" width="0.1524" layer="91"/>
</segment>
</net>
<net name="BIN1" class="0">
<segment>
<wire x1="-15.24" y1="48.26" x2="-10.16" y2="48.26" width="0.1524" layer="91"/>
<pinref part="BACK" gate="G$1" pin="11"/>
</segment>
<segment>
<pinref part="J1" gate="G$1" pin="13"/>
<wire x1="25.4" y1="63.5" x2="20.32" y2="63.5" width="0.1524" layer="91"/>
</segment>
</net>
<net name="BPWM" class="0">
<segment>
<wire x1="-15.24" y1="38.1" x2="-10.16" y2="38.1" width="0.1524" layer="91"/>
<pinref part="BACK" gate="G$1" pin="7"/>
</segment>
<segment>
<pinref part="J1" gate="G$1" pin="15"/>
<wire x1="25.4" y1="58.42" x2="20.32" y2="58.42" width="0.1524" layer="91"/>
</segment>
</net>
<net name="BIN2" class="0">
<segment>
<wire x1="-15.24" y1="27.94" x2="-10.16" y2="27.94" width="0.1524" layer="91"/>
<pinref part="BACK" gate="G$1" pin="3"/>
</segment>
<segment>
<pinref part="J1" gate="G$1" pin="17"/>
<wire x1="25.4" y1="53.34" x2="20.32" y2="53.34" width="0.1524" layer="91"/>
</segment>
</net>
<net name="LIN1" class="0">
<segment>
<wire x1="-15.24" y1="5.08" x2="-12.7" y2="5.08" width="0.1524" layer="91"/>
<pinref part="LEFT" gate="G$1" pin="11"/>
<wire x1="-12.7" y1="5.08" x2="-7.62" y2="5.08" width="0.1524" layer="91"/>
<junction x="-12.7" y="5.08"/>
</segment>
<segment>
<pinref part="J1" gate="G$1" pin="19"/>
<wire x1="25.4" y1="48.26" x2="20.32" y2="48.26" width="0.1524" layer="91"/>
</segment>
</net>
<net name="LPWM" class="0">
<segment>
<wire x1="-15.24" y1="-5.08" x2="-12.7" y2="-5.08" width="0.1524" layer="91"/>
<pinref part="LEFT" gate="G$1" pin="7"/>
<wire x1="-12.7" y1="-5.08" x2="-7.62" y2="-5.08" width="0.1524" layer="91"/>
<junction x="-12.7" y="-5.08"/>
</segment>
<segment>
<pinref part="J1" gate="G$1" pin="21"/>
<wire x1="25.4" y1="43.18" x2="20.32" y2="43.18" width="0.1524" layer="91"/>
</segment>
</net>
<net name="LIN2" class="0">
<segment>
<wire x1="-15.24" y1="-15.24" x2="-12.7" y2="-15.24" width="0.1524" layer="91"/>
<pinref part="LEFT" gate="G$1" pin="3"/>
<wire x1="-12.7" y1="-15.24" x2="-7.62" y2="-15.24" width="0.1524" layer="91"/>
<junction x="-12.7" y="-15.24"/>
</segment>
<segment>
<pinref part="J1" gate="G$1" pin="23"/>
<wire x1="25.4" y1="38.1" x2="20.32" y2="38.1" width="0.1524" layer="91"/>
</segment>
</net>
<net name="ARMIN1" class="0">
<segment>
<pinref part="J1" gate="G$1" pin="2"/>
<wire x1="45.72" y1="93.98" x2="50.8" y2="93.98" width="0.1524" layer="91"/>
</segment>
<segment>
<pinref part="ARM" gate="G$1" pin="3"/>
<wire x1="68.58" y1="127" x2="63.5" y2="127" width="0.1524" layer="91"/>
</segment>
</net>
<net name="ARMPWM" class="0">
<segment>
<pinref part="J1" gate="G$1" pin="4"/>
<wire x1="45.72" y1="88.9" x2="50.8" y2="88.9" width="0.1524" layer="91"/>
</segment>
<segment>
<pinref part="ARM" gate="G$1" pin="7"/>
<wire x1="68.58" y1="116.84" x2="63.5" y2="116.84" width="0.1524" layer="91"/>
</segment>
</net>
<net name="ARMIN2" class="0">
<segment>
<pinref part="J1" gate="G$1" pin="6"/>
<wire x1="45.72" y1="83.82" x2="50.8" y2="83.82" width="0.1524" layer="91"/>
</segment>
<segment>
<pinref part="ARM" gate="G$1" pin="11"/>
<wire x1="68.58" y1="106.68" x2="63.5" y2="106.68" width="0.1524" layer="91"/>
</segment>
</net>
<net name="EXTRAIN1" class="0">
<segment>
<pinref part="J1" gate="G$1" pin="8"/>
<wire x1="45.72" y1="78.74" x2="50.8" y2="78.74" width="0.1524" layer="91"/>
</segment>
<segment>
<pinref part="EXTRA" gate="G$1" pin="3"/>
<wire x1="68.58" y1="83.82" x2="63.5" y2="83.82" width="0.1524" layer="91"/>
</segment>
</net>
<net name="EXTRAPWM" class="0">
<segment>
<pinref part="J1" gate="G$1" pin="10"/>
<wire x1="45.72" y1="73.66" x2="50.8" y2="73.66" width="0.1524" layer="91"/>
</segment>
<segment>
<pinref part="EXTRA" gate="G$1" pin="7"/>
<wire x1="68.58" y1="73.66" x2="63.5" y2="73.66" width="0.1524" layer="91"/>
</segment>
</net>
<net name="EXTRAIN2" class="0">
<segment>
<pinref part="J1" gate="G$1" pin="12"/>
<wire x1="45.72" y1="68.58" x2="50.8" y2="68.58" width="0.1524" layer="91"/>
</segment>
<segment>
<pinref part="EXTRA" gate="G$1" pin="11"/>
<wire x1="68.58" y1="63.5" x2="63.5" y2="63.5" width="0.1524" layer="91"/>
</segment>
</net>
<net name="5V" class="0">
<segment>
<pinref part="J1" gate="G$1" pin="22"/>
<wire x1="45.72" y1="43.18" x2="50.8" y2="43.18" width="0.1524" layer="91"/>
</segment>
<segment>
<wire x1="60.96" y1="43.18" x2="58.42" y2="43.18" width="0.1524" layer="91"/>
<pinref part="IC1" gate="A" pin="I1"/>
</segment>
<segment>
<wire x1="60.96" y1="30.48" x2="58.42" y2="30.48" width="0.1524" layer="91"/>
<pinref part="IC1" gate="B" pin="I0"/>
</segment>
<segment>
<pinref part="ARMBRAKE" gate="1" pin="1"/>
<wire x1="134.62" y1="71.12" x2="134.62" y2="73.66" width="0.1524" layer="91"/>
</segment>
<segment>
<pinref part="BRAKEF" gate="1" pin="1"/>
<wire x1="134.62" y1="53.34" x2="134.62" y2="55.88" width="0.1524" layer="91"/>
</segment>
<segment>
<pinref part="BRAKEL" gate="1" pin="1"/>
<wire x1="134.62" y1="35.56" x2="134.62" y2="38.1" width="0.1524" layer="91"/>
</segment>
<segment>
<pinref part="BRAKEB" gate="1" pin="1"/>
<wire x1="134.62" y1="17.78" x2="134.62" y2="20.32" width="0.1524" layer="91"/>
</segment>
<segment>
<pinref part="BRAKER" gate="1" pin="1"/>
<wire x1="134.62" y1="0" x2="134.62" y2="2.54" width="0.1524" layer="91"/>
</segment>
</net>
<net name="BASEBRAKE" class="0">
<segment>
<pinref part="J1" gate="G$1" pin="24"/>
<wire x1="45.72" y1="38.1" x2="50.8" y2="38.1" width="0.1524" layer="91"/>
</segment>
<segment>
<wire x1="60.96" y1="25.4" x2="58.42" y2="25.4" width="0.1524" layer="91"/>
<pinref part="IC1" gate="B" pin="I1"/>
</segment>
</net>
<net name="ARMBRAKE" class="0">
<segment>
<pinref part="J1" gate="G$1" pin="20"/>
<wire x1="45.72" y1="48.26" x2="50.8" y2="48.26" width="0.1524" layer="91"/>
</segment>
<segment>
<wire x1="60.96" y1="48.26" x2="58.42" y2="48.26" width="0.1524" layer="91"/>
<pinref part="IC1" gate="A" pin="I0"/>
</segment>
</net>
<net name="N$4" class="0">
<segment>
<wire x1="76.2" y1="45.72" x2="78.74" y2="45.72" width="0.1524" layer="91"/>
<pinref part="IC2" gate="A" pin="I1"/>
<wire x1="78.74" y1="45.72" x2="78.74" y2="38.1" width="0.1524" layer="91"/>
<wire x1="78.74" y1="38.1" x2="86.36" y2="38.1" width="0.1524" layer="91"/>
<pinref part="IC1" gate="A" pin="O"/>
</segment>
</net>
<net name="N$7" class="0">
<segment>
<pinref part="IC2" gate="A" pin="I5"/>
<wire x1="76.2" y1="27.94" x2="86.36" y2="27.94" width="0.1524" layer="91"/>
<pinref part="IC2" gate="A" pin="I2"/>
<wire x1="86.36" y1="27.94" x2="86.36" y2="30.48" width="0.1524" layer="91"/>
<junction x="86.36" y="27.94"/>
<pinref part="IC2" gate="A" pin="I4"/>
<junction x="86.36" y="30.48"/>
<pinref part="IC1" gate="B" pin="O"/>
<pinref part="IC2" gate="A" pin="I3"/>
<wire x1="86.36" y1="33.02" x2="86.36" y2="35.56" width="0.1524" layer="91"/>
<wire x1="86.36" y1="30.48" x2="86.36" y2="33.02" width="0.1524" layer="91"/>
<junction x="86.36" y="33.02"/>
</segment>
</net>
<net name="GND" class="0">
<segment>
<pinref part="IC2" gate="A" pin="GND"/>
<wire x1="86.36" y1="20.32" x2="81.28" y2="20.32" width="0.1524" layer="91"/>
</segment>
<segment>
<pinref part="SUPPLY" gate="-2" pin="KL"/>
<wire x1="-106.68" y1="160.02" x2="-101.6" y2="160.02" width="0.1524" layer="91"/>
</segment>
<segment>
<pinref part="SUPPLY1" gate="-2" pin="KL"/>
<wire x1="-106.68" y1="119.38" x2="-101.6" y2="119.38" width="0.1524" layer="91"/>
</segment>
<segment>
<pinref part="SUPPLY2" gate="-2" pin="KL"/>
<wire x1="-106.68" y1="106.68" x2="-99.06" y2="106.68" width="0.1524" layer="91"/>
</segment>
<segment>
<pinref part="SUPPLY3" gate="-2" pin="KL"/>
<wire x1="-106.68" y1="93.98" x2="-101.6" y2="93.98" width="0.1524" layer="91"/>
</segment>
<segment>
<pinref part="SUPPLY4" gate="-2" pin="KL"/>
<wire x1="-106.68" y1="81.28" x2="-99.06" y2="81.28" width="0.1524" layer="91"/>
</segment>
<segment>
<pinref part="SUPPLY5" gate="-2" pin="KL"/>
<wire x1="-106.68" y1="68.58" x2="-99.06" y2="68.58" width="0.1524" layer="91"/>
</segment>
<segment>
<pinref part="SUPPLY6" gate="-2" pin="KL"/>
<wire x1="-106.68" y1="55.88" x2="-99.06" y2="55.88" width="0.1524" layer="91"/>
</segment>
</net>
<net name="N$1" class="0">
<segment>
<pinref part="IC2" gate="A" pin="O1"/>
<wire x1="111.76" y1="38.1" x2="111.76" y2="60.96" width="0.1524" layer="91"/>
<pinref part="ARMBRAKE" gate="1" pin="2"/>
<wire x1="111.76" y1="60.96" x2="134.62" y2="60.96" width="0.1524" layer="91"/>
</segment>
</net>
<net name="N$2" class="0">
<segment>
<pinref part="IC2" gate="A" pin="O2"/>
<wire x1="111.76" y1="35.56" x2="116.84" y2="35.56" width="0.1524" layer="91"/>
<wire x1="116.84" y1="35.56" x2="116.84" y2="43.18" width="0.1524" layer="91"/>
<pinref part="BRAKEF" gate="1" pin="2"/>
<wire x1="116.84" y1="43.18" x2="134.62" y2="43.18" width="0.1524" layer="91"/>
</segment>
</net>
<net name="N$3" class="0">
<segment>
<pinref part="IC2" gate="A" pin="O3"/>
<wire x1="111.76" y1="33.02" x2="129.54" y2="33.02" width="0.1524" layer="91"/>
<wire x1="129.54" y1="33.02" x2="129.54" y2="25.4" width="0.1524" layer="91"/>
<pinref part="BRAKEL" gate="1" pin="2"/>
<wire x1="129.54" y1="25.4" x2="134.62" y2="25.4" width="0.1524" layer="91"/>
</segment>
</net>
<net name="N$5" class="0">
<segment>
<pinref part="IC2" gate="A" pin="O4"/>
<wire x1="111.76" y1="30.48" x2="124.46" y2="30.48" width="0.1524" layer="91"/>
<wire x1="124.46" y1="30.48" x2="124.46" y2="7.62" width="0.1524" layer="91"/>
<pinref part="BRAKEB" gate="1" pin="2"/>
<wire x1="124.46" y1="7.62" x2="134.62" y2="7.62" width="0.1524" layer="91"/>
</segment>
</net>
<net name="N$6" class="0">
<segment>
<pinref part="IC2" gate="A" pin="O5"/>
<wire x1="111.76" y1="27.94" x2="121.92" y2="27.94" width="0.1524" layer="91"/>
<wire x1="121.92" y1="27.94" x2="121.92" y2="-10.16" width="0.1524" layer="91"/>
<pinref part="BRAKER" gate="1" pin="2"/>
<wire x1="121.92" y1="-10.16" x2="134.62" y2="-10.16" width="0.1524" layer="91"/>
</segment>
</net>
<net name="N$8" class="0">
<segment>
<pinref part="SUPPLY" gate="-1" pin="KL"/>
<pinref part="F1" gate="1" pin="1"/>
<wire x1="-106.68" y1="165.1" x2="-93.98" y2="165.1" width="0.1524" layer="91"/>
<wire x1="-93.98" y1="165.1" x2="-93.98" y2="154.94" width="0.1524" layer="91"/>
</segment>
</net>
<net name="N$9" class="0">
<segment>
<pinref part="F1" gate="1" pin="2"/>
<wire x1="-93.98" y1="144.78" x2="-93.98" y2="134.62" width="0.1524" layer="91"/>
<pinref part="KILL" gate="-2" pin="KL"/>
<wire x1="-93.98" y1="134.62" x2="-106.68" y2="134.62" width="0.1524" layer="91"/>
</segment>
</net>
<net name="N$10" class="0">
<segment>
<pinref part="SUPPLY1" gate="-1" pin="KL"/>
<pinref part="SUPPLY2" gate="-1" pin="KL"/>
<wire x1="-106.68" y1="124.46" x2="-106.68" y2="111.76" width="0.1524" layer="91"/>
<pinref part="SUPPLY3" gate="-1" pin="KL"/>
<wire x1="-106.68" y1="99.06" x2="-106.68" y2="111.76" width="0.1524" layer="91"/>
<junction x="-106.68" y="111.76"/>
<pinref part="SUPPLY4" gate="-1" pin="KL"/>
<wire x1="-106.68" y1="99.06" x2="-106.68" y2="86.36" width="0.1524" layer="91"/>
<junction x="-106.68" y="99.06"/>
<pinref part="SUPPLY5" gate="-1" pin="KL"/>
<wire x1="-106.68" y1="86.36" x2="-106.68" y2="73.66" width="0.1524" layer="91"/>
<junction x="-106.68" y="86.36"/>
<pinref part="SUPPLY6" gate="-1" pin="KL"/>
<wire x1="-106.68" y1="73.66" x2="-106.68" y2="60.96" width="0.1524" layer="91"/>
<junction x="-106.68" y="73.66"/>
<pinref part="KILL" gate="-1" pin="KL"/>
<wire x1="-106.68" y1="129.54" x2="-106.68" y2="124.46" width="0.1524" layer="91"/>
<junction x="-106.68" y="124.46"/>
</segment>
</net>
<net name="ARM1" class="0">
<segment>
<pinref part="ARMBRAKE" gate="2" pin="P"/>
<wire x1="152.4" y1="63.5" x2="152.4" y2="60.96" width="0.1524" layer="91"/>
</segment>
<segment>
<pinref part="X2" gate="-1" pin="KL"/>
<wire x1="200.66" y1="88.9" x2="205.74" y2="88.9" width="0.1524" layer="91"/>
<pinref part="X1" gate="-1" pin="KL"/>
<wire x1="200.66" y1="101.6" x2="205.74" y2="101.6" width="0.1524" layer="91"/>
<wire x1="205.74" y1="88.9" x2="205.74" y2="101.6" width="0.1524" layer="91"/>
</segment>
</net>
<net name="BRAKEF1" class="0">
<segment>
<pinref part="BRAKEF" gate="2" pin="P"/>
<wire x1="152.4" y1="45.72" x2="152.4" y2="40.64" width="0.1524" layer="91"/>
</segment>
<segment>
<pinref part="X3" gate="-1" pin="KL"/>
<wire x1="200.66" y1="73.66" x2="208.28" y2="73.66" width="0.1524" layer="91"/>
<wire x1="208.28" y1="73.66" x2="208.28" y2="58.42" width="0.1524" layer="91"/>
<pinref part="X4" gate="-1" pin="KL"/>
<wire x1="200.66" y1="58.42" x2="208.28" y2="58.42" width="0.1524" layer="91"/>
</segment>
</net>
<net name="BRAKEL1" class="0">
<segment>
<pinref part="BRAKEL" gate="2" pin="P"/>
<wire x1="152.4" y1="27.94" x2="152.4" y2="22.86" width="0.1524" layer="91"/>
</segment>
<segment>
<pinref part="X5" gate="-1" pin="KL"/>
<wire x1="200.66" y1="43.18" x2="210.82" y2="43.18" width="0.1524" layer="91"/>
<wire x1="210.82" y1="43.18" x2="210.82" y2="27.94" width="0.1524" layer="91"/>
<pinref part="X6" gate="-1" pin="KL"/>
<wire x1="210.82" y1="27.94" x2="200.66" y2="27.94" width="0.1524" layer="91"/>
</segment>
</net>
<net name="BRAKEB1" class="0">
<segment>
<pinref part="BRAKEB" gate="2" pin="P"/>
<wire x1="152.4" y1="10.16" x2="152.4" y2="5.08" width="0.1524" layer="91"/>
</segment>
<segment>
<pinref part="X7" gate="-1" pin="KL"/>
<wire x1="200.66" y1="12.7" x2="210.82" y2="12.7" width="0.1524" layer="91"/>
<wire x1="210.82" y1="12.7" x2="210.82" y2="-5.08" width="0.1524" layer="91"/>
<pinref part="X8" gate="-1" pin="KL"/>
<wire x1="210.82" y1="-5.08" x2="200.66" y2="-5.08" width="0.1524" layer="91"/>
</segment>
</net>
<net name="BRAKER1" class="0">
<segment>
<pinref part="BRAKER" gate="2" pin="P"/>
<wire x1="152.4" y1="-7.62" x2="152.4" y2="-12.7" width="0.1524" layer="91"/>
</segment>
<segment>
<pinref part="X9" gate="-1" pin="KL"/>
<wire x1="200.66" y1="-22.86" x2="210.82" y2="-22.86" width="0.1524" layer="91"/>
<pinref part="X10" gate="-1" pin="KL"/>
<wire x1="210.82" y1="-22.86" x2="210.82" y2="-40.64" width="0.1524" layer="91"/>
<wire x1="210.82" y1="-40.64" x2="200.66" y2="-40.64" width="0.1524" layer="91"/>
</segment>
</net>
<net name="ARM2" class="0">
<segment>
<pinref part="ARMBRAKE" gate="2" pin="S"/>
<wire x1="147.32" y1="71.12" x2="147.32" y2="78.74" width="0.1524" layer="91"/>
</segment>
<segment>
<pinref part="X1" gate="-2" pin="KL"/>
<wire x1="200.66" y1="96.52" x2="208.28" y2="96.52" width="0.1524" layer="91"/>
<wire x1="208.28" y1="96.52" x2="208.28" y2="83.82" width="0.1524" layer="91"/>
<pinref part="X2" gate="-2" pin="KL"/>
<wire x1="200.66" y1="83.82" x2="208.28" y2="83.82" width="0.1524" layer="91"/>
</segment>
</net>
<net name="BRAKEF2" class="0">
<segment>
<pinref part="BRAKEF" gate="2" pin="S"/>
<wire x1="147.32" y1="53.34" x2="147.32" y2="58.42" width="0.1524" layer="91"/>
</segment>
<segment>
<pinref part="X3" gate="-2" pin="KL"/>
<wire x1="200.66" y1="68.58" x2="205.74" y2="68.58" width="0.1524" layer="91"/>
<pinref part="X4" gate="-2" pin="KL"/>
<wire x1="200.66" y1="53.34" x2="205.74" y2="53.34" width="0.1524" layer="91"/>
<wire x1="205.74" y1="68.58" x2="205.74" y2="53.34" width="0.1524" layer="91"/>
</segment>
</net>
<net name="BRAKEL2" class="0">
<segment>
<pinref part="BRAKEL" gate="2" pin="S"/>
<wire x1="147.32" y1="35.56" x2="147.32" y2="40.64" width="0.1524" layer="91"/>
</segment>
<segment>
<pinref part="X5" gate="-2" pin="KL"/>
<wire x1="200.66" y1="38.1" x2="205.74" y2="38.1" width="0.1524" layer="91"/>
<pinref part="X6" gate="-2" pin="KL"/>
<wire x1="200.66" y1="22.86" x2="205.74" y2="22.86" width="0.1524" layer="91"/>
<wire x1="205.74" y1="38.1" x2="205.74" y2="22.86" width="0.1524" layer="91"/>
</segment>
</net>
<net name="BRAKEB2" class="0">
<segment>
<pinref part="BRAKEB" gate="2" pin="S"/>
<wire x1="147.32" y1="17.78" x2="147.32" y2="22.86" width="0.1524" layer="91"/>
</segment>
<segment>
<pinref part="X7" gate="-2" pin="KL"/>
<wire x1="200.66" y1="7.62" x2="205.74" y2="7.62" width="0.1524" layer="91"/>
<pinref part="X8" gate="-2" pin="KL"/>
<wire x1="200.66" y1="-10.16" x2="205.74" y2="-10.16" width="0.1524" layer="91"/>
<wire x1="205.74" y1="7.62" x2="205.74" y2="-10.16" width="0.1524" layer="91"/>
</segment>
</net>
<net name="BRAKER2" class="0">
<segment>
<pinref part="BRAKER" gate="2" pin="S"/>
<wire x1="147.32" y1="0" x2="147.32" y2="5.08" width="0.1524" layer="91"/>
</segment>
<segment>
<pinref part="X9" gate="-2" pin="KL"/>
<wire x1="200.66" y1="-27.94" x2="205.74" y2="-27.94" width="0.1524" layer="91"/>
<pinref part="X10" gate="-2" pin="KL"/>
<wire x1="200.66" y1="-45.72" x2="205.74" y2="-45.72" width="0.1524" layer="91"/>
<wire x1="205.74" y1="-27.94" x2="205.74" y2="-45.72" width="0.1524" layer="91"/>
</segment>
</net>
</nets>
</sheet>
</sheets>
<errors>
<approved hash="202,1,86.36,25.4,IC2,I6,,,,"/>
<approved hash="202,1,86.36,22.86,IC2,I7,,,,"/>
<approved hash="205,1,69.6935,45.7513,IC1P,GND,,,,"/>
<approved hash="205,1,69.6935,45.7513,IC1P,VCC,,,,"/>
</errors>
</schematic>
</drawing>
<compatibility>
<note version="8.2" severity="warning">
Since Version 8.2, EAGLE supports online libraries. The ids
of those online libraries will not be understood (or retained)
with this version.
</note>
<note version="8.3" severity="warning">
Since Version 8.3, EAGLE supports URNs for individual library
assets (packages, symbols, and devices). The URNs of those assets
will not be understood (or retained) with this version.
</note>
<note version="8.3" severity="warning">
Since Version 8.3, EAGLE supports the association of 3D packages
with devices in libraries, schematics, and board files. Those 3D
packages will not be understood (or retained) with this version.
</note>
</compatibility>
</eagle>
