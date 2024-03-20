import Image from 'next/image';
import { Schedule } from '@/components/schedule';

export default function Home() {
  return (
    <main className="font-body flex flex-col gap-16 mx-6 my-4">
      <div className="w-full flex flex-col items-center">
        <div className="flex gap-2 items-center">
          <h1 className="font-bold font-display text-[5rem]">DORA</h1>
          <Image src="/dora.png" alt="Dora" width={90} height={90} />
        </div>
        {/* <p>The Daycare Organising Robotic Assistant</p> */}
      </div>
      <div className="flex justify-center">
        <p>3D model here?</p>
      </div>
      <Schedule />
    </main>
  );
}
