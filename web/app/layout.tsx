import type { Metadata } from 'next';
import { Cabin_Sketch, Inter } from 'next/font/google';
import './globals.css';
import { Toaster } from '@/components/ui/toaster';

const cabinSketch = Cabin_Sketch({
  subsets: ['latin'],
  weight: ['400', '700'],
  variable: '--display-font',
});
const inter = Inter({ subsets: ['latin'], variable: '--body-font' });

export const metadata: Metadata = {
  title: 'DORA Web App',
  description: 'A web application for managing DORA, the Daycare Organising Robotic Assistant.',
};

export default function RootLayout({
  children,
}: Readonly<{
  children: React.ReactNode;
}>) {
  return (
    <html lang="en">
      <body className={`${cabinSketch.variable} ${inter.variable} h-screen overflow-y-hidden`}>
        {children}
      </body>
      <Toaster />
    </html>
  );
}
