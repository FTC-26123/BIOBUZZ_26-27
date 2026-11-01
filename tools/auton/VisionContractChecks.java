import org.firstinspires.ftc.teamcode.Auton.cycles.VisionRules;

public final class VisionContractChecks {
    private static void check(boolean value,String message){if(!value)throw new AssertionError(message);}
    public static void main(String[] args){
        check(VisionRules.POLLEN_PIPELINE==7,"Neural pipeline must remain 7");
        check(VisionRules.pollen("yellow_pollen",.8,0,-15),"Exact class accepted");
        for(String label:new String[]{"nectar","red_nectar","blue_nectar","pollen","Yellow_Pollen","yellow_pollen ",null})
            check(!VisionRules.pollen(label,.99,0,0),"Other labels rejected");
        check(!VisionRules.pollen("yellow_pollen",Double.NaN,0,0),"Invalid confidence rejected");
        check(!VisionRules.pollen("yellow_pollen",.9,Double.NaN,0),"Invalid angle rejected");
        check(!VisionRules.pollen("yellow_pollen",0,0,0),"Zero confidence rejected");
        check(VisionRules.freshTag(1000000000,900000000),"Fresh tag accepted");
        check(!VisionRules.freshTag(1000000000,800000000),"Stale tag rejected");
        check(!VisionRules.freshTag(1000000000,1000000001),"Future timestamp rejected");
        check(!VisionRules.freshTag(1000000000,0),"Missing timestamp rejected");
        for(int first:new int[]{30,34,38,42}){
            for(int id=30;id<46;id++)check(VisionRules.expectedTag(id,first)==(id>=first&&id<first+4),"Cell identity interlock");
        }
        System.out.println("Vision label, age and cell checks passed");
    }
}
