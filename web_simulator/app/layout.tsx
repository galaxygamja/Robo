import type { Metadata } from 'next';
import { Geist, Geist_Mono } from 'next/font/google';
import './globals.css';

const geistSans = Geist({
  variable: '--font-geist-sans',
  subsets: ['latin'],
});

const geistMono = Geist_Mono({
  variable: '--font-geist-mono',
  subsets: ['latin'],
});

export const metadata: Metadata = {
  title: 'Robo 구조팀 · 드론 없이도 작동하는 예선 시뮬레이터',
  description:
    '고정 카메라 2대를 기본으로 햄스터 2대와 비버 2대의 디스크·색별 원기둥·의료키트 운반을 모의하며, 박쥐 드론 모드도 비교할 수 있는 예선 시뮬레이터',
};

export default function RootLayout({
  children,
}: Readonly<{
  children: React.ReactNode;
}>) {
  return (
    <html lang="ko">
      <body
        className={`${geistSans.variable} ${geistMono.variable} antialiased`}
      >
        {children}
      </body>
    </html>
  );
}
